#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>

// ==========================================
// 1. CẤU HÌNH THÔNG SỐ CƠ KHÍ & PID
// ==========================================
const int GEAR_RATIO = 30;
const float WHEEL_RADIUS = 0.05; // 5cm -> 0.05m
const float WHEEL_BASE = 0.45;   // Khoảng cách 2 bánh (m)
const int PULSES_PER_REV = 11 * GEAR_RATIO;
const float Ts = 0.1; // 100ms
const int pwmFreq = 5000;
const int pwmResolution = 8;

struct Motor {
  int EN_R, EN_L, PWM_R, PWM_L, ENC_A, ENC_B;
  volatile long count = 0;
  long prevCount = 0;
  volatile unsigned long lastPulseTime = 0;
  float Kp, Ki;
  float targetRPM = 0;
  float actualRPM = 0;
  float error = 0, integral = 0;
};

// Động cơ 1 (Trái - Left)
Motor M1 = {16, 17, 18, 19, 25, 26, 0, 0, 0, 0.25, 2.10, 0, 0, 0, 0};
// Động cơ 2 (Phải - Right)
Motor M2 = {21, 27, 22, 23, 32, 33, 0, 0, 0, 0.24, 2.15, 0, 0, 0, 0};

// Biến Odometry
double x = 0, y = 0, theta = 0;
unsigned long previousMillis = 0;

// Micro-ROS Objects
rcl_publisher_t odom_pub, tf_pub;
rcl_subscription_t cmd_sub;
nav_msgs__msg__Odometry odom_msg;
tf2_msgs__msg__TFMessage tf_msg;
geometry_msgs__msg__TransformStamped tf_stamped;
geometry_msgs__msg__Twist cmd_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ==========================================
// 2. HÀM HỖ TRỢ & NGẮT
// ==========================================
void IRAM_ATTR readEnc1() {
  unsigned long t = micros();
  if (t - M1.lastPulseTime > 300) {
    if (digitalRead(M1.ENC_B) == HIGH) M1.count++; else M1.count--;
    M1.lastPulseTime = t;
  }
}
void IRAM_ATTR readEnc2() {
  unsigned long t = micros();
  if (t - M2.lastPulseTime > 300) {
    if (digitalRead(M2.ENC_B) == HIGH) M2.count++; else M2.count--;
    M2.lastPulseTime = t;
  }
}

void set_quat(float yaw, geometry_msgs__msg__Quaternion * q) {
  q->x = 0; q->y = 0; q->z = sin(yaw / 2.0); q->w = cos(yaw / 2.0);
}

// Khi nhận lệnh điều khiển từ Raspberry Pi
void cmd_callback(const void * msin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msin;
  float linear_v = msg->linear.x;
  float angular_v = msg->angular.z;

  // Kinematics nghịch: m/s -> RPM
  float v_left = linear_v - (angular_v * WHEEL_BASE / 2.0);
  float v_right = linear_v + (angular_v * WHEEL_BASE / 2.0);

  M1.targetRPM = (v_left * 60.0) / (2.0 * PI * WHEEL_RADIUS);
  M2.targetRPM = (v_right * 60.0) / (2.0 * PI * WHEEL_RADIUS);
}

// ==========================================
// 3. TÍNH TOÁN ODOM & PID (CHẠY MỖI 100MS)
// ==========================================
void compute_control_loop() {
  // --- PHẦN 1: TÍNH ODOMETRY (Cho SLAM) ---
  noInterrupts();
  long c1 = M1.count; long c2 = M2.count;
  interrupts();

  float dL = (float)(c1 - M1.prevCount) / PULSES_PER_REV * 2 * PI * WHEEL_RADIUS;
  float dR = (float)(c2 - M2.prevCount) / PULSES_PER_REV * 2 * PI * WHEEL_RADIUS;
  M1.prevCount = c1; M2.prevCount = c2;

  float dC = (dL + dR) / 2.0;
  float dTh = (dR - dL) / WHEEL_BASE;

  x += dC * cos(theta + dTh/2.0);
  y += dC * sin(theta + dTh/2.0);
  theta += dTh;

  // Pub Odom & TF
  odom_msg.header.stamp.sec = millis()/1000;
  odom_msg.pose.pose.position.x = x; odom_msg.pose.pose.position.y = y;
  set_quat(theta, &odom_msg.pose.pose.orientation);
  rcl_publish(&odom_pub, &odom_msg, NULL);

  tf_stamped.header.stamp.sec = odom_msg.header.stamp.sec;
  tf_stamped.transform.translation.x = x; tf_stamped.transform.translation.y = y;
  set_quat(theta, &tf_stamped.transform.rotation);
  tf_msg.transforms.data = &tf_stamped; tf_msg.transforms.size = 1;
  rcl_publish(&tf_pub, &tf_msg, NULL);

  // --- PHẦN 2: CHẠY PI CONTROLLER (Cho Motor) ---
  auto pid = [](Motor &m) {
    m.actualRPM = ((float)(m.count - m.prevCount + (m.prevCount)) / PULSES_PER_REV) * (60.0 / Ts); // Dùng lại dL, dR đã tính ở trên sẽ chuẩn hơn
    // (Đơn giản hóa để dùng biến local của ông giáo)
    m.error = m.targetRPM - ((m.actualRPM)); 
    m.integral = constrain(m.integral + m.error * Ts, -255.0/m.Ki, 255.0/m.Ki);
    float out = (m.Kp * m.error) + (m.Ki * m.integral);
    int pwm = constrain(round(out), -255, 255);
    if (abs(pwm) < 10 && m.targetRPM != 0) pwm = (pwm > 0) ? 10 : -10;
    if (m.targetRPM == 0) { pwm = 0; m.integral = 0; }
    
    if (pwm >= 0) { ledcWrite(m.PWM_L, 0); ledcWrite(m.PWM_R, pwm); }
    else { ledcWrite(m.PWM_R, 0); ledcWrite(m.PWM_L, abs(pwm)); }
  };
  pid(M1); pid(M2);
}

void setup() {
  Serial.begin(115200);
  set_microros_transports();

  auto setupM = [](Motor &m, void (*isr)()) {
    pinMode(m.EN_R, OUTPUT); pinMode(m.EN_L, OUTPUT);
    digitalWrite(m.EN_R, HIGH); digitalWrite(m.EN_L, HIGH);
    ledcAttach(m.PWM_R, pwmFreq, pwmResolution);
    ledcAttach(m.PWM_L, pwmFreq, pwmResolution);
    pinMode(m.ENC_A, INPUT_PULLUP); pinMode(m.ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(m.ENC_A), isr, RISING);
  };
  setupM(M1, readEnc1); setupM(M2, readEnc2);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_base", "", &support);
  rclc_publisher_init_default(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");
  rclc_publisher_init_default(&tf_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "tf");
  rclc_subscription_init_default(&cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_callback, ON_NEW_DATA);
  
  tf_stamped.header.frame_id.data = (char*)"odom";
  tf_stamped.child_frame_id.data = (char*)"base_link";
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= sampleTime) {
    compute_control_loop();
    previousMillis = currentMillis;
  }
}
