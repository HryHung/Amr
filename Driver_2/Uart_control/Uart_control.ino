#include <Arduino.h>
#include <math.h>

#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

// =====================================================
// CẤU HÌNH CƠ BẢN
// =====================================================
const int GEAR_RATIO = 30;
const float MAX_RPM = 333.0f;
const int PULSES_PER_REV = 11 * GEAR_RATIO;
const unsigned long sampleTime = 100;   // 10 Hz

const uint32_t pwmFreq = 5000;
const uint8_t pwmResolution = 8;
const int PWM_MAX = 255;

const unsigned long debounceDelay = 300;   // us

const float TARGET_ZERO_EPS_RPM = 1.0f;
const float FF_MAX_FRACTION = 0.85f;
const float PI_INTEGRAL_FRACTION = 0.60f;

const unsigned long CMD_VEL_TIMEOUT_MS = 300;

// =====================================================
// THÔNG SỐ HÌNH HỌC ROBOT
// =====================================================
const float WHEEL_RADIUS_M = 0.06f;
const float WHEEL_BASE_M   = 0.435f;

// =====================================================
// ROS TOPIC
// =====================================================
const char * CMD_VEL_TOPIC = "cmd_vel";
const char * RPM_FEEDBACK_TOPIC = "wheel_rpm_feedback";

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

  float Kp, Ki, Kd;
  float Kff;
  float ffOffset;
  float ffTau;
  float ffFiltered;

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

// M1 = Left
Motor M1 = {
  16, 17, 18, 19, 25, 26,
  0, 0, 0,
  0.48f, 1.55f, 0.0f,
  0.62f, 12.0f, 0.12f, 0.0f,
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
  0, 0,
  1, 1
};

// M2 = Right
Motor M2 = {
  21, 22, 23, 27, 33, 32,
  0, 0, 0,
  0.50f, 1.43f, 0.0f,
  0.65f, 9.0f, 0.12f, 0.0f,
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
  0, 0,
  -1, -1
};

unsigned long previousControlMillis = 0;
unsigned long lastCmdVelMillis = 0;

// =====================================================
// MICRO-ROS OBJECTS
// =====================================================
rcl_publisher_t rpm_pub;
rcl_subscription_t cmd_sub;

std_msgs__msg__Float32MultiArray rpm_msg;
geometry_msgs__msg__Twist cmd_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

float rpm_feedback_buffer[2] = {0.0f, 0.0f};

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
    ledcWriteChannel(chL, 0);
    ledcWriteChannel(chR, signedCmd);
  } else if (signedCmd < 0) {
    ledcWriteChannel(chR, 0);
    ledcWriteChannel(chL, -signedCmd);
  } else {
    ledcWriteChannel(chR, 0);
    ledcWriteChannel(chL, 0);
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

  const float v = (float)msg->linear.x;
  const float w = (float)msg->angular.z;

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

  m.actualRPM = ((float)diffCount / (float)PULSES_PER_REV) * (60.0f / Ts);

  if (fabsf(m.targetRPM) < TARGET_ZERO_EPS_RPM) {
    m.error = 0.0f;
    m.integral = 0.0f;
    m.prevError = 0.0f;
    m.derivative = 0.0f;
    m.ffFiltered = 0.0f;
    m.lastPWM = 0;
    return 0;
  }

  const float ffLimit = FF_MAX_FRACTION * PWM_MAX;

  float ffRaw = m.Kff * (fabsf(m.targetRPM) / MAX_RPM) * ffLimit;
  ffRaw += m.ffOffset;
  ffRaw = clampf(ffRaw, 0.0f, ffLimit);
  ffRaw *= signf_nonzero(m.targetRPM);

  const float alpha = Ts / (m.ffTau + Ts);
  m.ffFiltered += alpha * (ffRaw - m.ffFiltered);

  m.error = m.targetRPM - m.actualRPM;
  m.derivative = (m.error - m.prevError) / Ts;

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

  if (abs(pwm) < 8) {
    pwm = (pwm >= 0) ? 8 : -8;
  }

  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);

  m.prevError = m.error;
  m.lastPWM = pwm;

  return pwm;
}

void publish_rpm_feedback() {
  rpm_feedback_buffer[0] = M1.actualRPM;
  rpm_feedback_buffer[1] = M2.actualRPM;
  rcl_publish(&rpm_pub, &rpm_msg, NULL);
}

// =====================================================
// CONTROL LOOP
// =====================================================
void compute_control_loop() {
  const float Ts = sampleTime / 1000.0f;

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

  int pwm1 = compute_motor_pwm(M1, diff1, Ts);
  int pwm2 = compute_motor_pwm(M2, diff2, Ts);

  apply_motor_pwm(M1, pwm1);
  apply_motor_pwm(M2, pwm2);

  publish_rpm_feedback();

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

  bool okR = ledcAttachChannel(m.PWM_R, pwmFreq, pwmResolution, get_pwm_r_channel(m));
  bool okL = ledcAttachChannel(m.PWM_L, pwmFreq, pwmResolution, get_pwm_l_channel(m));

  if (!okR || !okL) {
    while (1) {
      delay(100);
    }
  }

  ledcWriteChannel(get_pwm_r_channel(m), 0);
  ledcWriteChannel(get_pwm_l_channel(m), 0);

  attachInterrupt(digitalPinToInterrupt(m.ENC_A), isr, RISING);
}

// =====================================================
// SETUP MICRO-ROS MSG
// =====================================================
void setup_ros_messages() {
  std_msgs__msg__Float32MultiArray__init(&rpm_msg);
  geometry_msgs__msg__Twist__init(&cmd_msg);

  rpm_msg.data.data = rpm_feedback_buffer;
  rpm_msg.data.size = 2;
  rpm_msg.data.capacity = 2;

  rpm_msg.layout.dim.data = NULL;
  rpm_msg.layout.dim.size = 0;
  rpm_msg.layout.dim.capacity = 0;
  rpm_msg.layout.data_offset = 0;
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  set_microros_transports();

  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
    delay(500);
  }

  reset_motor_runtime(M1);
  reset_motor_runtime(M2);

  setup_motor(M1, readEnc1);
  setup_motor(M2, readEnc2);

  setup_ros_messages();

  allocator = rcl_get_default_allocator();

  rcl_ret_t rc;

  rc = rclc_support_init(&support, 0, NULL, &allocator);
  if (rc != RCL_RET_OK) {
    while (1) {
      delay(100);
    }
  }

  rc = rclc_node_init_default(&node, "esp32_base", "", &support);
  if (rc != RCL_RET_OK) {
    while (1) {
      delay(100);
    }
  }

  rc = rclc_publisher_init_default(
    &rpm_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    RPM_FEEDBACK_TOPIC
  );
  if (rc != RCL_RET_OK) {
    while (1) {
      delay(100);
    }
  }

  rc = rclc_subscription_init_default(
    &cmd_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    CMD_VEL_TOPIC
  );
  if (rc != RCL_RET_OK) {
    while (1) {
      delay(100);
    }
  }

  rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
  if (rc != RCL_RET_OK) {
    while (1) {
      delay(100);
    }
  }

  rc = rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_callback, ON_NEW_DATA);
  if (rc != RCL_RET_OK) {
    while (1) {
      delay(100);
    }
  }

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
