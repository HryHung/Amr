#include <Arduino.h>
#include <math.h>

// ==========================================
// CẤU HÌNH THÔNG SỐ CƠ BẢN
// ==========================================
const int GEAR_RATIO = 30;                  // 333 RPM version
const float MAX_RPM = 333.0f;
const int PULSES_PER_REV = 11 * GEAR_RATIO; // 330 xung / vòng bánh (CẦN XÁC MINH THỰC TẾ)
const unsigned long sampleTime = 100;       // ms
const int pwmFreq = 5000;
const int pwmResolution = 8;
const int PWM_MAX = 255;
const unsigned long debounceDelay = 300;    // us

// Tốc độ debug
float linearRPMSet = 60.0f;   // q/z thay đổi giá trị này
float turnRPMSet   = 70.0f;   // quay tại chỗ
const float speedStepRPM = 10.0f;

// Ngưỡng xem như target = 0
const float RPM_ZERO_EPS = 0.5f;

// ==========================================
// CẤU TRÚC MOTOR
// ==========================================
struct Motor {
  int EN_R, EN_L, PWM_R, PWM_L, ENC_A, ENC_B;

  volatile long count = 0;
  long prevCount = 0;
  volatile unsigned long lastPulseTime = 0;

  float Kp, Ki;
  float targetRPM = 0.0f;
  float actualRPM = 0.0f;
  float error = 0.0f;
  float integral = 0.0f;

  int lastPWM = 0;
  long lastDiff = 0;

  int minPwmFwd;
  int minPwmRev;

  int pwmSign;
  int encSign;
};

// ==========================================
// MAPPING CHÂN THEO BẢNG VIẾT TAY
// ==========================================

// ML (Motor Left)
Motor M1 = {
  16, 17, 18, 19, 25, 26,
  0, 0, 0,
  0.48f, 1.55f,   
  0, 0, 0, 0,
  0, 0,
  18, 24,
  1, 1
};

// MR (Motor Right)
Motor M2 = {
  21, 22, 23, 27, 33, 32,
  0, 0, 0,
  0.50f, 1.43f,  
  0, 0, 0, 0,
  0, 0,
  18, 24,
  -1, -1
};

unsigned long previousMillis = 0;
bool verboseLog = true;

// ==========================================
// MODE ĐIỀU KHIỂN DEBUG
// ==========================================
enum DriveMode {
  MODE_STOP = 0,
  MODE_FORWARD,
  MODE_BACKWARD,
  MODE_TURN_LEFT,
  MODE_TURN_RIGHT,
  MODE_M1_FWD_ONLY,
  MODE_M1_REV_ONLY,
  MODE_M2_FWD_ONLY,
  MODE_M2_REV_ONLY
};

DriveMode currentMode = MODE_STOP;

// ==========================================
// HÀM TIỆN ÍCH
// ==========================================
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

const char* modeToString(DriveMode mode) {
  switch (mode) {
    case MODE_STOP:        return "STOP";
    case MODE_FORWARD:     return "FORWARD";
    case MODE_BACKWARD:    return "BACKWARD";
    case MODE_TURN_LEFT:   return "TURN_LEFT";
    case MODE_TURN_RIGHT:  return "TURN_RIGHT";
    case MODE_M1_FWD_ONLY: return "M1_FWD_ONLY";
    case MODE_M1_REV_ONLY: return "M1_REV_ONLY";
    case MODE_M2_FWD_ONLY: return "M2_FWD_ONLY";
    case MODE_M2_REV_ONLY: return "M2_REV_ONLY";
    default:               return "UNKNOWN";
  }
}

void printHelp() {
  Serial.println();
  Serial.println("===== SERIAL DEBUG TELEOP =====");
  Serial.println("w : di tien");
  Serial.println("s : di lui");
  Serial.println("a : quay trai tai cho");
  Serial.println("d : quay phai tai cho");
  Serial.println("x : dung");
  Serial.println("q : tang linear RPM");
  Serial.println("z : giam linear RPM");
  Serial.println("j : test M1 tien");
  Serial.println("n : test M1 lui");
  Serial.println("k : test M2 tien");
  Serial.println("m : test M2 lui");
  Serial.println("p : in trang thai hien tai");
  Serial.println("v : bat/tat log lien tuc");
  Serial.println("h : help");
  Serial.println("================================");
  Serial.println();
}

void printStatus() {
  Serial.printf(
    "[MODE=%s | linearRPM=%.1f | turnRPM=%.1f | verbose=%d]\n",
    modeToString(currentMode), linearRPMSet, turnRPMSet, verboseLog ? 1 : 0
  );

  Serial.printf(
    "M1 -> pins[%d,%d,%d,%d,%d,%d] | T=%7.1f | A=%7.1f | E=%7.1f | PWM=%4d | dCnt=%5ld | pwmSign=%2d | encSign=%2d\n",
    M1.EN_R, M1.EN_L, M1.PWM_R, M1.PWM_L, M1.ENC_A, M1.ENC_B,
    M1.targetRPM, M1.actualRPM, M1.error, M1.lastPWM, M1.lastDiff, M1.pwmSign, M1.encSign
  );

  Serial.printf(
    "M2 -> pins[%d,%d,%d,%d,%d,%d] | T=%7.1f | A=%7.1f | E=%7.1f | PWM=%4d | dCnt=%5ld | pwmSign=%2d | encSign=%2d\n",
    M2.EN_R, M2.EN_L, M2.PWM_R, M2.PWM_L, M2.ENC_A, M2.ENC_B,
    M2.targetRPM, M2.actualRPM, M2.error, M2.lastPWM, M2.lastDiff, M2.pwmSign, M2.encSign
  );
}

// ==========================================
// GÁN TARGET THEO MODE
// ==========================================
void applyTargetsFromMode() {
  switch (currentMode) {
    case MODE_STOP:
      M1.targetRPM = 0.0f;
      M2.targetRPM = 0.0f;
      break;

    case MODE_FORWARD:
      M1.targetRPM = linearRPMSet;
      M2.targetRPM = linearRPMSet;
      break;

    case MODE_BACKWARD:
      M1.targetRPM = -linearRPMSet;
      M2.targetRPM = -linearRPMSet;
      break;

    case MODE_TURN_LEFT:
      M1.targetRPM = -turnRPMSet;
      M2.targetRPM =  turnRPMSet;
      break;

    case MODE_TURN_RIGHT:
      M1.targetRPM =  turnRPMSet;
      M2.targetRPM = -turnRPMSet;
      break;

    case MODE_M1_FWD_ONLY:
      M1.targetRPM = linearRPMSet;
      M2.targetRPM = 0.0f;
      break;

    case MODE_M1_REV_ONLY:
      M1.targetRPM = -linearRPMSet;
      M2.targetRPM = 0.0f;
      break;

    case MODE_M2_FWD_ONLY:
      M1.targetRPM = 0.0f;
      M2.targetRPM = linearRPMSet;
      break;

    case MODE_M2_REV_ONLY:
      M1.targetRPM = 0.0f;
      M2.targetRPM = -linearRPMSet;
      break;
  }

  M1.targetRPM = clampf(M1.targetRPM, -MAX_RPM, MAX_RPM);
  M2.targetRPM = clampf(M2.targetRPM, -MAX_RPM, MAX_RPM);
}

// ==========================================
// XUẤT PWM RA DRIVER
// ==========================================
void applyMotorPWM(Motor &m, int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);

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
// PI CHO MỖI MOTOR
// ==========================================
void computePI(Motor &m, float dt) {
  noInterrupts();
  long currentCount = m.count;
  interrupts();

  long countDiff = currentCount - m.prevCount;
  m.prevCount = currentCount;
  m.lastDiff = countDiff;

  m.actualRPM = ((float)countDiff / (float)PULSES_PER_REV) * (60.0f / dt);

  if (fabsf(m.targetRPM) < RPM_ZERO_EPS) {
    m.error = 0.0f;
    m.integral = 0.0f;
    applyMotorPWM(m, 0);
    return;
  }

  m.error = m.targetRPM - m.actualRPM;

  float maxIntegral = 255.0f / m.Ki;
  float newIntegral = m.integral + (m.error * dt);
  newIntegral = clampf(newIntegral, -maxIntegral, maxIntegral);

  float outUnsat = (m.Kp * m.error) + (m.Ki * newIntegral);
  float outSat = clampf(outUnsat, -255.0f, 255.0f);

  bool allowIntegral = false;
  if (fabsf(outUnsat - outSat) < 1e-4f) {
    allowIntegral = true;
  } else if (outUnsat > 255.0f && m.error < 0.0f) {
    allowIntegral = true;
  } else if (outUnsat < -255.0f && m.error > 0.0f) {
    allowIntegral = true;
  }

  if (allowIntegral) {
    m.integral = newIntegral;
  }

  float output = (m.Kp * m.error) + (m.Ki * m.integral);
  int finalPWM = (int)lroundf(clampf(output, -255.0f, 255.0f));

  int minPWM = (m.targetRPM > 0.0f) ? m.minPwmFwd : m.minPwmRev;

  if (finalPWM == 0 && fabsf(m.targetRPM) >= 1.0f) {
    finalPWM = (m.targetRPM > 0.0f) ? minPWM : -minPWM;
  } else if (abs(finalPWM) < minPWM) {
    finalPWM = (m.targetRPM > 0.0f) ? minPWM : -minPWM;
  }

  applyMotorPWM(m, finalPWM);
}

// ==========================================
// XỬ LÝ LỆNH SERIAL
// ==========================================
void processCommand(char c) {
  switch (c) {
    case 'w':
    case 'W':
      currentMode = MODE_FORWARD;
      applyTargetsFromMode();
      Serial.println(">> CMD: FORWARD");
      break;

    case 's':
    case 'S':
      currentMode = MODE_BACKWARD;
      applyTargetsFromMode();
      Serial.println(">> CMD: BACKWARD");
      break;

    case 'a':
    case 'A':
      currentMode = MODE_TURN_LEFT;
      applyTargetsFromMode();
      Serial.println(">> CMD: TURN_LEFT");
      break;

    case 'd':
    case 'D':
      currentMode = MODE_TURN_RIGHT;
      applyTargetsFromMode();
      Serial.println(">> CMD: TURN_RIGHT");
      break;

    case 'x':
    case 'X':
      currentMode = MODE_STOP;
      applyTargetsFromMode();
      Serial.println(">> CMD: STOP");
      break;

    case 'q':
    case 'Q':
      linearRPMSet = clampf(linearRPMSet + speedStepRPM, 0.0f, MAX_RPM);
      applyTargetsFromMode();
      Serial.printf(">> linearRPMSet = %.1f\n", linearRPMSet);
      break;

    case 'z':
    case 'Z':
      linearRPMSet = clampf(linearRPMSet - speedStepRPM, 0.0f, MAX_RPM);
      applyTargetsFromMode();
      Serial.printf(">> linearRPMSet = %.1f\n", linearRPMSet);
      break;

    case 'j':
    case 'J':
      currentMode = MODE_M1_FWD_ONLY;
      applyTargetsFromMode();
      Serial.println(">> CMD: M1_FWD_ONLY");
      break;

    case 'n':
    case 'N':
      currentMode = MODE_M1_REV_ONLY;
      applyTargetsFromMode();
      Serial.println(">> CMD: M1_REV_ONLY");
      break;

    case 'k':
    case 'K':
      currentMode = MODE_M2_FWD_ONLY;
      applyTargetsFromMode();
      Serial.println(">> CMD: M2_FWD_ONLY");
      break;

    case 'm':
    case 'M':
      currentMode = MODE_M2_REV_ONLY;
      applyTargetsFromMode();
      Serial.println(">> CMD: M2_REV_ONLY");
      break;

    case 'p':
    case 'P':
      printStatus();
      break;

    case 'v':
    case 'V':
      verboseLog = !verboseLog;
      Serial.printf(">> verboseLog = %d\n", verboseLog ? 1 : 0);
      break;

    case 'h':
    case 'H':
      printHelp();
      break;

    case '\r':
    case '\n':
    case ' ':
      break;

    default:
      Serial.printf(">> Unknown cmd: %c\n", c);
      break;
  }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("HE THONG DEBUG 2 DONG CO - SERIAL TELEOP + PI");
  Serial.println("Arduino IDE Serial Monitor: 115200, No line ending");
  printHelp();

  setupMotor(M1, readEnc1);
  setupMotor(M2, readEnc2);

  currentMode = MODE_STOP;
  applyTargetsFromMode();

  previousMillis = millis();

  printStatus();
}

// ==========================================
// LOOP
// ==========================================
void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    processCommand(c);
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= sampleTime) {
    float dt = (currentMillis - previousMillis) / 1000.0f;
    previousMillis = currentMillis;

    computePI(M1, dt);
    computePI(M2, dt);

    if (verboseLog) {
      Serial.printf(
        "[%s | lin=%.1f | turn=%.1f]  ",
        modeToString(currentMode), linearRPMSet, turnRPMSet
      );

      Serial.printf(
        "M1(T=%6.1f A=%7.1f E=%7.1f PWM=%4d d=%5ld)  ",
        M1.targetRPM, M1.actualRPM, M1.error, M1.lastPWM, M1.lastDiff
      );

      Serial.printf(
        "M2(T=%6.1f A=%7.1f E=%7.1f PWM=%4d d=%5ld)\n",
        M2.targetRPM, M2.actualRPM, M2.error, M2.lastPWM, M2.lastDiff
      );
    }
  }
}