#include <Arduino.h>
#include <math.h>
#include <time.h>

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

// =====================================================
// CẤU HÌNH CƠ BẢN
// =====================================================
const int GEAR_RATIO = 30;
const float MAX_RPM = 333.0f;
const int PULSES_PER_REV = 11 * GEAR_RATIO;   // XÁC MINH THỰC TẾ NẾU CẦN
const unsigned long sampleTime = 100;         // ms
const int pwmFreq = 5000;
const int pwmResolution = 8;
const int PWM_MAX = 255;
const unsigned long debounceDelay = 300;      // us

// Feedforward / safety
const float TARGET_ZERO_EPS_RPM = 1.0f;
const float FF_MAX_FRACTION = 0.85f;
const float PI_INTEGRAL_FRACTION = 0.60f;

// Safety for ROS cmd_vel
const unsigned long CMD_VEL_TIMEOUT_MS = 300;

// =====================================================
// THÔNG SỐ HÌNH HỌC ROBOT - PHẢI ĐIỀN ĐÚNG
// =====================================================
const float WHEEL_RADIUS_M = 0.06f;
const float WHEEL_BASE_M   = 0.435f;

// =====================================================
// ROS TOPIC / FRAME
// =====================================================
const char * CMD_VEL_TOPIC = "cmd_vel";
const char * ODOM_TOPIC    = "odom";
const char * ODOM_FRAME    = "odom";
const char * BASE_FRAME    = "base_link";

// =====================================================
// PWM CHANNELS ESP32
// =====================================================
const int M1_PWM_R_CH = 0;
const int M1_PWM_L_CH = 1;
const int M2_PWM_R_CH = 2;
const int M2_PWM_L_CH = 3;

// =====================================================
// CẤU TRÚC MOTOR
// =====================================================
struct Motor {
  int EN_R, EN_L, PWM_R, PWM_L, ENC_A, ENC_B;

  volatile long count;
  long prevCount;
  volatile unsigned long lastPulseTime;

  // PI/PID
  float Kp, Ki, Kd;

  // Feedforward
  float Kff;
  float ffOffset;
  float ffTau;
  float ffFiltered;

  // Runtime
  float targetRPM;
  float actualRPM;
  float error;
  float integral;
  float prevError;
  float derivative;

  long lastDiff;
  int lastPWM;

  int pwmSign;
  int encSign;
};

// =====================================================
// MAPPING CHÂN ĐÃ CHỐT
// M1 = Left
// M2 = Right
// =====================================================

// M1 = Left
Motor M1 = {
  16, 17, 18, 19, 25, 26,
  0, 0, 0,

  // PI
  0.48f, 1.55f, 0.0f,

  // Feedforward
  0.62f, 12.0f, 0.12f, 0.0f,

  // Runtime
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,

  0, 0,
  1, 1
};

// M2 = Right
Motor M2 = {
  21, 22, 23, 27, 33, 32,
  0, 0, 0,

  // PI
  0.50f, 1.43f, 0.0f,

  // Feedforward
  0.65f, 9.0f, 0.12f, 0.0f,

  // Runtime
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,

  0, 0,
  -1, -1
};

// =====================================================
// ODOM STATE
// =====================================================
float x_m = 0.0f;
float y_m = 0.0f;
float theta_rad = 0.0f;

unsigned long previousControlMillis = 0;
unsigned long lastCmdVelMillis = 0;

// =====================================================
// MICRO-ROS OBJECTS
// =====================================================
rcl_publisher_t odom_pub;
rcl_publisher_t tf_pub;
rcl_subscription_t cmd_sub;

nav_msgs__msg__Odometry odom_msg;
tf2_msgs__msg__TFMessage tf_msg;
geometry_msgs__msg__TransformStamped tf_stamped;
geometry_msgs__msg__Twist cmd_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// =====================================================
// HÀM TIỆN ÍCH
// =====================================================
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline int get_pwm_r_channel(const Motor &m) {
  return (&m == &M1) ? M1_PWM_R_CH : M2_PWM_R_CH;
}

static inline int get_pwm_l_channel(const Motor &m) {
  return (&m == &M1) ? M1_PWM_L_CH : M2_PWM_L_CH;
}

static inline float signf_nonzero(float x) {
  return (x >= 0.0f) ? 1.0f : -1.0f;
}

static inline float normalize_angle(float a) {
  while (a > PI)  a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

void set_quat_from_yaw(float yaw, geometry_msgs__msg__Quaternion *q) {
  q->x = 0.0;
  q->y = 0.0;
  q->z = sinf(yaw * 0.5f);
  q->w = cosf(yaw * 0.5f);
}

void set_ros_string(rosidl_runtime_c__String *str, const char *text) {
  if (str->data != NULL) {
    rosidl_runtime_c__String__fini(str);
  }
  rosidl_runtime_c__String__init(str);
  rosidl_runtime_c__String__assign(str, text);
}

void fill_stamp(builtin_interfaces__msg__Time *stamp) {
  unsigned long ms = millis();
  stamp->sec = (int32_t)(ms / 1000UL);
  stamp->nanosec = (uint32_t)((ms % 1000UL) * 1000000UL);
}

void stop_all_targets() {
  M1.targetRPM = 0.0f;
  M2.targetRPM = 0.0f;
}

void apply_motor_pwm(const Motor &m, int pwmCmdSigned) {
  int signedCmd = m.pwmSign * pwmCmdSigned;
  signedCmd = constrain(signedCmd, -PWM_MAX, PWM_MAX);

  int chR = get_pwm_r_channel(m);
  int chL = get_pwm_l_channel(m);

  if (signedCmd > 0) {
    ledcWrite(chL, 0);
    ledcWrite(chR, signedCmd);
  } else if (signedCmd < 0) {
    ledcWrite(chR, 0);
    ledcWrite(chL, -signedCmd);
  } else {
    ledcWrite(chR, 0);
    ledcWrite(chL, 0);
  }
}

void reset_motor_runtime(Motor &m) {
  m.targetRPM   = 0.0f;
  m.actualRPM   = 0.0f;
  m.error       = 0.0f;
  m.integral    = 0.0f;
  m.prevError   = 0.0f;
  m.derivative  = 0.0f;
  m.ffFiltered  = 0.0f;
  m.lastDiff    = 0;
  m.lastPWM     = 0;
}

// =====================================================
// ISR ENCODER
// =====================================================
void IRAM_ATTR readEnc1() {
  unsigned long t = micros();
  if (t - M1.lastPulseTime > debounceDelay) {
    int dir = (digitalRead(M1.ENC_B) == HIGH) ? 1 : -1;
    M1.count += M1.encSign * dir;
    M1.lastPulseTime = t;
  }
}

void IRAM_ATTR readEnc2() {
  unsigned long t = micros();
  if (t - M2.lastPulseTime > debounceDelay) {
    int dir = (digitalRead(M2.ENC_B) == HIGH) ? 1 : -1;
    M2.count += M2.encSign * dir;
    M2.lastPulseTime = t;
  }
}

// =====================================================
// CMD_VEL CALLBACK
// =====================================================
void cmd_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  const float v = (float)msg->linear.x;   // m/s
  const float w = (float)msg->angular.z;  // rad/s

  const float v_left  = v - (w * WHEEL_BASE_M * 0.5f);
  const float v_right = v + (w * WHEEL_BASE_M * 0.5f);

  float leftRPM  = (v_left  * 60.0f) / (2.0f * PI * WHEEL_RADIUS_M);
  float rightRPM = (v_right * 60.0f) / (2.0f * PI * WHEEL_RADIUS_M);

  M1.targetRPM = clampf(leftRPM,  -MAX_RPM, MAX_RPM);
  M2.targetRPM = clampf(rightRPM, -MAX_RPM, MAX_RPM);

  lastCmdVelMillis = millis();
}

// =====================================================
// PID + FEEDFORWARD
// =====================================================
int compute_motor_pwm(Motor &m, long diffCount, float Ts) {
  m.lastDiff = diffCount;

  // RPM phản hồi
  m.actualRPM = ((float)diffCount / (float)PULSES_PER_REV) * (60.0f / Ts);

  // Nếu target rất nhỏ => ép dừng sạch
  if (fabsf(m.targetRPM) < TARGET_ZERO_EPS_RPM) {
    m.error = 0.0f;
    m.integral = 0.0f;
    m.prevError = 0.0f;
    m.derivative = 0.0f;
    m.ffFiltered = 0.0f;
    m.lastPWM = 0;
    return 0;
  }

  // Feedforward thô theo RPM đặt
  const float ffLimit = FF_MAX_FRACTION * PWM_MAX;

  float ffRaw = m.Kff * (fabsf(m.targetRPM) / MAX_RPM) * ffLimit;
  ffRaw += m.ffOffset;
  ffRaw = clampf(ffRaw, 0.0f, ffLimit);
  ffRaw *= signf_nonzero(m.targetRPM);

  // Low-pass feedforward
  const float alpha = Ts / (m.ffTau + Ts);
  m.ffFiltered += alpha * (ffRaw - m.ffFiltered);

  // PID
  m.error = m.targetRPM - m.actualRPM;
  m.derivative = (m.error - m.prevError) / Ts;

  // Giới hạn phần tích phân theo fraction của PWM_MAX
  const float integralTermLimit = PI_INTEGRAL_FRACTION * PWM_MAX;
  if (m.Ki > 1e-6f) {
    const float integralStateLimit = integralTermLimit / m.Ki;
    m.integral += m.error * Ts;
    m.integral = clampf(m.integral, -integralStateLimit, integralStateLimit);
  } else {
    m.integral = 0.0f;
  }

  float pTerm = m.Kp * m.error;
  float iTerm = m.Ki * m.integral;
  float dTerm = m.Kd * m.derivative;

  float u = m.ffFiltered + pTerm + iTerm + dTerm;
  u = clampf(u, -PWM_MAX, PWM_MAX);

  int pwm = (int)lroundf(u);

  // Deadband nhẹ: chỉ khi có target mà PWM quá nhỏ
  if (abs(pwm) < 8) {
    pwm = (pwm >= 0) ? 8 : -8;
  }

  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);

  m.prevError = m.error;
  m.lastPWM = pwm;

  return pwm;
}

// =====================================================
// ODOM + TF PUBLISH
// =====================================================
void publish_odom_and_tf(float dL, float dR, float Ts) {
  const float dC = 0.5f * (dL + dR);
  const float dTh = (dR - dL) / WHEEL_BASE_M;

  x_m += dC * cosf(theta_rad + 0.5f * dTh);
  y_m += dC * sinf(theta_rad + 0.5f * dTh);
  theta_rad = normalize_angle(theta_rad + dTh);

  fill_stamp(&odom_msg.header.stamp);

  odom_msg.pose.pose.position.x = x_m;
  odom_msg.pose.pose.position.y = y_m;
  odom_msg.pose.pose.position.z = 0.0;
  set_quat_from_yaw(theta_rad, &odom_msg.pose.pose.orientation);

  odom_msg.twist.twist.linear.x = dC / Ts;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = dTh / Ts;

  rcl_publish(&odom_pub, &odom_msg, NULL);

  tf_stamped.header.stamp = odom_msg.header.stamp;
  tf_stamped.transform.translation.x = x_m;
  tf_stamped.transform.translation.y = y_m;
  tf_stamped.transform.translation.z = 0.0;
  set_quat_from_yaw(theta_rad, &tf_stamped.transform.rotation);

  tf_msg.transforms.data = &tf_stamped;
  tf_msg.transforms.size = 1;
  tf_msg.transforms.capacity = 1;

  rcl_publish(&tf_pub, &tf_msg, NULL);
}

// =====================================================
// CONTROL LOOP
// =====================================================
void compute_control_loop() {
  const float Ts = sampleTime / 1000.0f;

  // Safety timeout cmd_vel
  if ((millis() - lastCmdVelMillis) > CMD_VEL_TIMEOUT_MS) {
    stop_all_targets();
  }

  long cur1, cur2;
  noInterrupts();
  cur1 = M1.count;
  cur2 = M2.count;
  interrupts();

  long diff1 = cur1 - M1.prevCount;
  long diff2 = cur2 - M2.prevCount;

  // Odom từ encoder
  const float dL = ((float)diff1 / (float)PULSES_PER_REV) * (2.0f * PI * WHEEL_RADIUS_M);
  const float dR = ((float)diff2 / (float)PULSES_PER_REV) * (2.0f * PI * WHEEL_RADIUS_M);

  publish_odom_and_tf(dL, dR, Ts);

  // PID + Feedforward
  int pwm1 = compute_motor_pwm(M1, diff1, Ts);
  int pwm2 = compute_motor_pwm(M2, diff2, Ts);

  apply_motor_pwm(M1, pwm1);
  apply_motor_pwm(M2, pwm2);

  M1.prevCount = cur1;
  M2.prevCount = cur2;
}

// =====================================================
// MOTOR SETUP
// =====================================================
void setup_motor(Motor &m, void (*isr)()) {
  pinMode(m.EN_R, OUTPUT);
  pinMode(m.EN_L, OUTPUT);
  digitalWrite(m.EN_R, HIGH);
  digitalWrite(m.EN_L, HIGH);

  pinMode(m.ENC_A, INPUT_PULLUP);
  pinMode(m.ENC_B, INPUT_PULLUP);

  ledcSetup(get_pwm_r_channel(m), pwmFreq, pwmResolution);
  ledcSetup(get_pwm_l_channel(m), pwmFreq, pwmResolution);

  ledcAttachPin(m.PWM_R, get_pwm_r_channel(m));
  ledcAttachPin(m.PWM_L, get_pwm_l_channel(m));

  ledcWrite(get_pwm_r_channel(m), 0);
  ledcWrite(get_pwm_l_channel(m), 0);

  attachInterrupt(digitalPinToInterrupt(m.ENC_A), isr, RISING);
}

// =====================================================
// SETUP MICRO-ROS MSG
// =====================================================
void setup_ros_messages() {
  nav_msgs__msg__Odometry__init(&odom_msg);
  tf2_msgs__msg__TFMessage__init(&tf_msg);
  geometry_msgs__msg__TransformStamped__init(&tf_stamped);
  geometry_msgs__msg__Twist__init(&cmd_msg);

  set_ros_string(&odom_msg.header.frame_id, ODOM_FRAME);
  set_ros_string(&odom_msg.child_frame_id, BASE_FRAME);

  set_ros_string(&tf_stamped.header.frame_id, ODOM_FRAME);
  set_ros_string(&tf_stamped.child_frame_id, BASE_FRAME);

  // Covariance cơ bản
  for (int i = 0; i < 36; i++) {
    odom_msg.pose.covariance[i] = 0.0;
    odom_msg.twist.covariance[i] = 0.0;
  }

  odom_msg.pose.covariance[0]  = 1e-3;
  odom_msg.pose.covariance[7]  = 1e-3;
  odom_msg.pose.covariance[14] = 1e6;
  odom_msg.pose.covariance[21] = 1e6;
  odom_msg.pose.covariance[28] = 1e6;
  odom_msg.pose.covariance[35] = 1e-2;

  odom_msg.twist.covariance[0]  = 1e-2;
  odom_msg.twist.covariance[7]  = 1e-2;
  odom_msg.twist.covariance[14] = 1e6;
  odom_msg.twist.covariance[21] = 1e6;
  odom_msg.twist.covariance[28] = 1e6;
  odom_msg.twist.covariance[35] = 1e-1;

  tf_msg.transforms.data = &tf_stamped;
  tf_msg.transforms.size = 1;
  tf_msg.transforms.capacity = 1;
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  set_microros_transports();

  reset_motor_runtime(M1);
  reset_motor_runtime(M2);

  setup_motor(M1, readEnc1);
  setup_motor(M2, readEnc2);

  setup_ros_messages();

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_base", "", &support);

  rclc_publisher_init_default(
    &odom_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    ODOM_TOPIC
  );

  rclc_publisher_init_default(
    &tf_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "tf"
  );

  rclc_subscription_init_default(
    &cmd_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    CMD_VEL_TOPIC
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_callback, ON_NEW_DATA);

  previousControlMillis = millis();
  lastCmdVelMillis = millis();
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

  unsigned long now = millis();
  if (now - previousControlMillis >= sampleTime) {
    previousControlMillis += sampleTime;
    compute_control_loop();
  }
}
