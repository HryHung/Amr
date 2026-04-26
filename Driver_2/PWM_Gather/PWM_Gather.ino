#include <Arduino.h>
#include <math.h>

// ==========================================
// CẤU HÌNH THÔNG SỐ CƠ BẢN
// ==========================================
const int GEAR_RATIO = 30;
const float MAX_RPM = 333.0f;
const int PULSES_PER_REV = 11 * GEAR_RATIO;   // CẦN XÁC MINH THỰC TẾ
const unsigned long sampleTime = 100;         // ms
const int pwmFreq = 5000;
const int pwmResolution = 8;
const int PWM_MAX = 255;
const unsigned long debounceDelay = 300;      // us

// Kịch bản sweep
const unsigned long PRE_DELAY_MS = 1000;      // đứng yên trước test
const unsigned long PEAK_HOLD_MS = 2000;      // giữ 100% trong 2s
const int PWM_STEP_PERCENT = 1;               // bước tăng 1%

// ==========================================
// CẤU TRÚC MOTOR
// ==========================================
struct Motor {
  int EN_R, EN_L, PWM_R, PWM_L, ENC_A, ENC_B;

  volatile long count = 0;
  long prevCount = 0;
  volatile unsigned long lastPulseTime = 0;

  // Để sẵn cho bước verify PID sau này
  float Kp = 0.0f;
  float Ki = 0.0f;
  float Kd = 0.0f;

  float actualRPM = 0.0f;
  long lastDiff = 0;
  int lastPWM = 0;

  int pwmSign;
  int encSign;
};

// ==========================================
// MAPPING CHÂN THEO BẢNG ĐÃ CHỐT
// L = M1 làm chuẩn
// R = M2 đảo hướng logic
// ==========================================

// Motor L
Motor M1 = {
  16, 17, 18, 19, 25, 26,
  0, 0, 0,
  0.0f, 0.0f, 0.0f,
  0.0f, 0, 0,
  1, 1
};

// Motor R
Motor M2 = {
  21, 22, 23, 27, 33, 32,
  0, 0, 0,
  0.0f, 0.0f, 0.0f,
  0.0f, 0, 0,
  -1, -1
};

// ==========================================
// BIẾN ĐIỀU KHIỂN TEST
// ==========================================
enum SweepState {
  IDLE = 0,
  PRE_DELAY,
  RAMP_UP,
  HOLD_PEAK,
  DONE
};

SweepState sweepState = IDLE;

unsigned long testStartMillis = 0;
unsigned long stateStartMillis = 0;
unsigned long previousSampleMillis = 0;

int pwmPercentCmd = 0;

// ==========================================
// HÀM TIỆN ÍCH
// ==========================================
int percentToPwm(int percent) {
  percent = constrain(percent, 0, 100);
  return (int)lround((percent / 100.0f) * PWM_MAX);
}

void stopMotor(Motor &m) {
  ledcWrite(m.PWM_R, 0);
  ledcWrite(m.PWM_L, 0);
  m.lastPWM = 0;
}

void stopAllMotors() {
  stopMotor(M1);
  stopMotor(M2);
}

void applyMotorPWM(Motor &m, int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);

  // đổi chiều logic theo motor
  pwm *= m.pwmSign;

  if (pwm > 0) {
    ledcWrite(m.PWM_L, 0);
    ledcWrite(m.PWM_R, pwm);
  } else if (pwm < 0) {
    ledcWrite(m.PWM_R, 0);
    ledcWrite(m.PWM_L, -pwm);
  } else {
    ledcWrite(m.PWM_R, 0);
    ledcWrite(m.PWM_L, 0);
  }

  m.lastPWM = pwm;
}

void resetEncoderState(Motor &m) {
  noInterrupts();
  m.count = 0;
  interrupts();

  m.prevCount = 0;
  m.lastPulseTime = 0;
  m.actualRPM = 0.0f;
  m.lastDiff = 0;
  m.lastPWM = 0;
}

void resetAllStates() {
  resetEncoderState(M1);
  resetEncoderState(M2);
  stopAllMotors();
  pwmPercentCmd = 0;
}

void printHeader() {
  Serial.println("==============================================");
  Serial.println("PWM SWEEP LOGGER FOR PID IDENTIFICATION");
  Serial.println("Command: press 's' to start test");
  Serial.println("Robot must be on stand, no-load only");
  Serial.println("Columns: Time_ms | PWM_percent | RPM_Motor_R | RPM_Motor_L");
  Serial.println("Kp Ki Kd are defined but not used in this sweep");
  Serial.println("==============================================");
  Serial.println("Time_ms,PWM_percent,RPM_Motor_R,RPM_Motor_L");
}

// ==========================================
// ENCODER ISR
// ==========================================
void IRAM_ATTR readEnc1() {
  unsigned long t = micros();
  if (t - M1.lastPulseTime > debounceDelay) {
    if (digitalRead(M1.ENC_B) == HIGH) M1.count += M1.encSign;
    else                               M1.count -= M1.encSign;
    M1.lastPulseTime = t;
  }
}

void IRAM_ATTR readEnc2() {
  unsigned long t = micros();
  if (t - M2.lastPulseTime > debounceDelay) {
    if (digitalRead(M2.ENC_B) == HIGH) M2.count += M2.encSign;
    else                               M2.count -= M2.encSign;
    M2.lastPulseTime = t;
  }
}

// ==========================================
// SETUP MOTOR
// ==========================================
void setupMotor(Motor &m, void (*isr)()) {
  pinMode(m.EN_R, OUTPUT);
  pinMode(m.EN_L, OUTPUT);
  digitalWrite(m.EN_R, HIGH);
  digitalWrite(m.EN_L, HIGH);

  ledcAttach(m.PWM_R, pwmFreq, pwmResolution);
  ledcAttach(m.PWM_L, pwmFreq, pwmResolution);
  ledcWrite(m.PWM_R, 0);
  ledcWrite(m.PWM_L, 0);

  pinMode(m.ENC_A, INPUT_PULLUP);
  pinMode(m.ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(m.ENC_A), isr, RISING);
}

// ==========================================
// CẬP NHẬT RPM
// ==========================================
void updateMotorRPM(Motor &m, float dtSec) {
  noInterrupts();
  long currentCount = m.count;
  interrupts();

  long diff = currentCount - m.prevCount;
  m.prevCount = currentCount;
  m.lastDiff = diff;

  m.actualRPM = ((float)diff / (float)PULSES_PER_REV) * (60.0f / dtSec);
}

// ==========================================
// BẮT ĐẦU TEST
// ==========================================
void startSweepTest() {
  resetAllStates();

  testStartMillis = millis();
  stateStartMillis = testStartMillis;
  previousSampleMillis = testStartMillis;

  sweepState = PRE_DELAY;

  Serial.println();
  Serial.println("EVENT,START_SWEEP");
  Serial.println("EVENT,STATE,PRE_DELAY");
  printHeader();
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  setupMotor(M1, readEnc1);
  setupMotor(M2, readEnc2);

  stopAllMotors();

  Serial.println();
  Serial.println("PWM SWEEP LOGGER READY");
  Serial.println("Press 's' to start");
  Serial.println("Motor_L = M1, Motor_R = M2");
  Serial.println("M2 is inverted logically to match robot forward convention");
  Serial.printf("M1 PID default: Kp=%.3f Ki=%.3f Kd=%.3f\n", M1.Kp, M1.Ki, M1.Kd);
  Serial.printf("M2 PID default: Kp=%.3f Ki=%.3f Kd=%.3f\n", M2.Kp, M2.Ki, M2.Kd);
}

// ==========================================
// LOOP
// ==========================================
void loop() {
  // Nhận lệnh serial
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == 's' || c == 'S') {
      if (sweepState == IDLE || sweepState == DONE) {
        startSweepTest();
      } else {
        Serial.println("EVENT,TEST_ALREADY_RUNNING");
      }
    }

    if (c == 'x' || c == 'X') {
      stopAllMotors();
      sweepState = IDLE;
      Serial.println("EVENT,ABORT");
    }
  }

  if (sweepState == IDLE || sweepState == DONE) {
    return;
  }

  unsigned long currentMillis = millis();

  if (currentMillis - previousSampleMillis < sampleTime) {
    return;
  }

  float dtSec = (currentMillis - previousSampleMillis) / 1000.0f;
  previousSampleMillis = currentMillis;

  // =========================
  // STATE MACHINE
  // =========================
  if (sweepState == PRE_DELAY) {
    pwmPercentCmd = 0;
    applyMotorPWM(M1, 0);
    applyMotorPWM(M2, 0);

    if (currentMillis - stateStartMillis >= PRE_DELAY_MS) {
      sweepState = RAMP_UP;
      stateStartMillis = currentMillis;
      pwmPercentCmd = 0;
      Serial.println("EVENT,STATE,RAMP_UP");
    }
  }
  else if (sweepState == RAMP_UP) {
    int pwmCmd = percentToPwm(pwmPercentCmd);
    applyMotorPWM(M1, pwmCmd);
    applyMotorPWM(M2, pwmCmd);

    if (pwmPercentCmd < 100) {
      pwmPercentCmd += PWM_STEP_PERCENT;
      if (pwmPercentCmd > 100) pwmPercentCmd = 100;
    } else {
      sweepState = HOLD_PEAK;
      stateStartMillis = currentMillis;
      Serial.println("EVENT,STATE,HOLD_PEAK");
    }
  }
  else if (sweepState == HOLD_PEAK) {
    int pwmCmd = percentToPwm(100);
    applyMotorPWM(M1, pwmCmd);
    applyMotorPWM(M2, pwmCmd);

    pwmPercentCmd = 100;

    if (currentMillis - stateStartMillis >= PEAK_HOLD_MS) {
      sweepState = DONE;
      stopAllMotors();
      Serial.println("EVENT,STATE,DONE");
      Serial.println("EVENT,END_SWEEP");
    }
  }

  // cập nhật đo lường sau khi đã áp lệnh
  updateMotorRPM(M1, dtSec);
  updateMotorRPM(M2, dtSec);

  // Xuất đúng cột yêu cầu:
  // Time | %PWM | RPM Motor_R | RPM Motor_L
  unsigned long t = currentMillis - testStartMillis;
  Serial.printf("%lu,%d,%.3f,%.3f\n", t, pwmPercentCmd, M2.actualRPM, M1.actualRPM);
}