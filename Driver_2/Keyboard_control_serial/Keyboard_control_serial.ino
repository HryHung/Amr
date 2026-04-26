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

// Teleop step
const int CMD_STEP_PERCENT = 5;

// Feedforward / safety
const float TARGET_ZERO_EPS_RPM = 1.0f;
const float FF_MAX_FRACTION = 0.85f;      // feedforward không chiếm toàn bộ PWM
const float PI_INTEGRAL_FRACTION = 0.60f; // giới hạn phần tích phân

// ==========================================
// TELEOP MODE
// ==========================================
enum MotionCommand {
  CMD_STOP = 0,
  CMD_FORWARD,
  CMD_BACKWARD,
  CMD_TURN_LEFT,
  CMD_TURN_RIGHT
};

// ==========================================
// CẤU TRÚC MOTOR
// ==========================================
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

// ==========================================
// MAPPING CHÂN ĐÃ CHỐT
// M1 = Left
// M2 = Right
// ==========================================

// M1 = Left
Motor M1 = {
  16, 17, 18, 19, 25, 26,
  0, 0, 0,

  // PID
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

  // PID
  0.50f, 1.43f, 0.0f,

  // Feedforward
  0.65f, 9.0f, 0.12f, 0.0f,

  // Runtime
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,

  0, 0,
  -1, -1
};

// ==========================================
// BIẾN CHUNG
// ==========================================
MotionCommand currentCmd = CMD_STOP;
unsigned long previousSampleMillis = 0;

// Mức lệnh người dùng chỉnh bằng q/z
int speedPercentCmd = 30;

// target từng bánh theo teleop
float targetLeftRPM = 0.0f;
float targetRightRPM = 0.0f;

// ==========================================
// HÀM TIỆN ÍCH
// ==========================================
float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float signf_local(float x) {
  if (x > 0.0f) return 1.0f;
  if (x < 0.0f) return -1.0f;
  return 0.0f;
}

int percentToPwm(int percent) {
  percent = constrain(percent, 0, 100);
  return (int)lround((percent / 100.0f) * PWM_MAX);
}

float percentToRPM(int percent) {
  percent = constrain(percent, 0, 100);
  return (percent / 100.0f) * MAX_RPM;
}

const char* cmdToString(MotionCommand cmd) {
  switch (cmd) {
    case CMD_STOP:      return "STOP";
    case CMD_FORWARD:   return "FORWARD";
    case CMD_BACKWARD:  return "BACKWARD";
    case CMD_TURN_LEFT: return "TURN_LEFT";
    case CMD_TURN_RIGHT:return "TURN_RIGHT";
    default:            return "UNKNOWN";
  }
}

void printHelp() {
  Serial.println();
  Serial.println("============= TELEOP PI + FEEDFORWARD =============");
  Serial.println("w : tien");
  Serial.println("s : lui");
  Serial.println("a : quay trai tai cho");
  Serial.println("d : quay phai tai cho");
  Serial.println("x : dung");
  Serial.println("q : tang toc (% moi lan bam)");
  Serial.println("z : giam toc (% moi lan bam)");
  Serial.println("p : in thong so PI + FF");
  Serial.println("h : help");
  Serial.println("---------------------------------------------------");
  Serial.println("Dat Serial Monitor: 115200, No line ending");
  Serial.println("===================================================");
  Serial.println();
}

void printPIDStatus() {
  Serial.printf("M1 Left  PI: Kp=%.4f Ki=%.4f Kd=%.4f | FF: Kff=%.4f Off=%.2f Tau=%.3f\n",
                M1.Kp, M1.Ki, M1.Kd, M1.Kff, M1.ffOffset, M1.ffTau);
  Serial.printf("M2 Right PI: Kp=%.4f Ki=%.4f Kd=%.4f | FF: Kff=%.4f Off=%.2f Tau=%.3f\n",
                M2.Kp, M2.Ki, M2.Kd, M2.Kff, M2.ffOffset, M2.ffTau);
}

void printCommandStatus() {
  Serial.printf("[CMD] %s | speed=%d%% | target=%.1f RPM | L=%.1f | R=%.1f\n",
                cmdToString(currentCmd),
                speedPercentCmd,
                percentToRPM(speedPercentCmd),
                targetLeftRPM,
                targetRightRPM);
}

void resetMotorControl(Motor &m) {
  noInterrupts();
  m.count = 0;
  interrupts();

  m.prevCount = 0;
  m.lastPulseTime = 0;

  m.ffFiltered = 0.0f;

  m.targetRPM = 0.0f;
  m.actualRPM = 0.0f;
  m.error = 0.0f;
  m.integral = 0.0f;
  m.prevError = 0.0f;
  m.derivative = 0.0f;

  m.lastDiff = 0;
  m.lastPWM = 0;
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

void setupMotorHardware(Motor &m, void (*isr)()) {
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

void updateMotorRPM(Motor &m, float dtSec) {
  noInterrupts();
  long currentCount = m.count;
  interrupts();

  long diff = currentCount - m.prevCount;
  m.prevCount = currentCount;
  m.lastDiff = diff;

  if (dtSec > 0.0f) {
    m.actualRPM = ((float)diff / (float)PULSES_PER_REV) * (60.0f / dtSec);
  } else {
    m.actualRPM = 0.0f;
  }
}

void updateBothRPM(float dtSec) {
  updateMotorRPM(M1, dtSec);
  updateMotorRPM(M2, dtSec);
}

// ==========================================
// FEEDFORWARD
// ==========================================
float computeFilteredFeedforward(Motor &m, float targetRPM, float dtSec) {
  if (fabsf(targetRPM) < TARGET_ZERO_EPS_RPM || dtSec <= 0.0f) {
    m.ffFiltered = 0.0f;
    return 0.0f;
  }

  float rawFF = m.Kff * fabsf(targetRPM) + m.ffOffset;
  rawFF = clampf(rawFF, 0.0f, FF_MAX_FRACTION * PWM_MAX);
  rawFF *= signf_local(targetRPM);

  float alpha = dtSec / (m.ffTau + dtSec);
  alpha = clampf(alpha, 0.0f, 1.0f);

  m.ffFiltered += alpha * (rawFF - m.ffFiltered);
  return m.ffFiltered;
}

// ==========================================
// PI + FEEDFORWARD
// ==========================================
void runPIDFF(Motor &m, float targetRPM, float dtSec) {
  m.targetRPM = clampf(targetRPM, -MAX_RPM, MAX_RPM);

  if (fabsf(m.targetRPM) < TARGET_ZERO_EPS_RPM) {
    m.error = 0.0f;
    m.integral = 0.0f;
    m.prevError = 0.0f;
    m.derivative = 0.0f;
    m.ffFiltered = 0.0f;
    applyMotorPWM(m, 0);
    return;
  }

  m.error = m.targetRPM - m.actualRPM;

  if (dtSec > 0.0f) {
    m.derivative = (m.error - m.prevError) / dtSec;
  } else {
    m.derivative = 0.0f;
  }

  float ffTerm = computeFilteredFeedforward(m, m.targetRPM, dtSec);

  float maxIntegral = 0.0f;
  float newIntegral = m.integral;
  bool allowIntegral = false;

  if (fabsf(m.Ki) > 1e-6f) {
    maxIntegral = (PI_INTEGRAL_FRACTION * PWM_MAX) / fabsf(m.Ki);
    newIntegral = clampf(m.integral + m.error * dtSec, -maxIntegral, maxIntegral);

    float uUnsat = ffTerm + (m.Kp * m.error) + (m.Ki * newIntegral) + (m.Kd * m.derivative);
    float uSat = clampf(uUnsat, -PWM_MAX, PWM_MAX);

    if (fabsf(uUnsat - uSat) < 1e-4f) {
      allowIntegral = true;
    } else if (uUnsat > PWM_MAX && m.error < 0.0f) {
      allowIntegral = true;
    } else if (uUnsat < -PWM_MAX && m.error > 0.0f) {
      allowIntegral = true;
    }
  }

  if (allowIntegral) {
    m.integral = newIntegral;
  }

  float output = ffTerm + (m.Kp * m.error) + (m.Ki * m.integral) + (m.Kd * m.derivative);
  int pwm = (int)lroundf(clampf(output, -PWM_MAX, PWM_MAX));

  applyMotorPWM(m, pwm);
  m.prevError = m.error;
}

// ==========================================
// ISR ENCODER
// ==========================================
void IRAM_ATTR readEnc1() {
  unsigned long t = micros();
  if (t - M1.lastPulseTime > debounceDelay) {
    if (digitalRead(M1.ENC_B) == HIGH) {
      M1.count += M1.encSign;
    } else {
      M1.count -= M1.encSign;
    }
    M1.lastPulseTime = t;
  }
}

void IRAM_ATTR readEnc2() {
  unsigned long t = micros();
  if (t - M2.lastPulseTime > debounceDelay) {
    if (digitalRead(M2.ENC_B) == HIGH) {
      M2.count += M2.encSign;
    } else {
      M2.count -= M2.encSign;
    }
    M2.lastPulseTime = t;
  }
}

// ==========================================
// TELEOP LOGIC
// ==========================================
void updateTargetsFromCommand() {
  float baseRPM = percentToRPM(speedPercentCmd);

  switch (currentCmd) {
    case CMD_STOP:
      targetLeftRPM = 0.0f;
      targetRightRPM = 0.0f;
      break;

    case CMD_FORWARD:
      targetLeftRPM = +baseRPM;
      targetRightRPM = +baseRPM;
      break;

    case CMD_BACKWARD:
      targetLeftRPM = -baseRPM;
      targetRightRPM = -baseRPM;
      break;

    case CMD_TURN_LEFT:
      targetLeftRPM = -baseRPM;
      targetRightRPM = +baseRPM;
      break;

    case CMD_TURN_RIGHT:
      targetLeftRPM = +baseRPM;
      targetRightRPM = -baseRPM;
      break;

    default:
      targetLeftRPM = 0.0f;
      targetRightRPM = 0.0f;
      break;
  }
}

void stopCommand() {
  currentCmd = CMD_STOP;
  updateTargetsFromCommand();

  M1.integral = 0.0f;
  M1.prevError = 0.0f;
  M1.ffFiltered = 0.0f;

  M2.integral = 0.0f;
  M2.prevError = 0.0f;
  M2.ffFiltered = 0.0f;

  stopAllMotors();
  printCommandStatus();
}

// ==========================================
// SERIAL
// ==========================================
void processImmediateCommand(char c) {
  switch (c) {
    case 'w':
    case 'W':
      currentCmd = CMD_FORWARD;
      updateTargetsFromCommand();
      printCommandStatus();
      break;

    case 's':
    case 'S':
      currentCmd = CMD_BACKWARD;
      updateTargetsFromCommand();
      printCommandStatus();
      break;

    case 'a':
    case 'A':
      currentCmd = CMD_TURN_LEFT;
      updateTargetsFromCommand();
      printCommandStatus();
      break;

    case 'd':
    case 'D':
      currentCmd = CMD_TURN_RIGHT;
      updateTargetsFromCommand();
      printCommandStatus();
      break;

    case 'x':
    case 'X':
      stopCommand();
      break;

    case 'q':
    case 'Q':
      speedPercentCmd += CMD_STEP_PERCENT;
      if (speedPercentCmd > 100) speedPercentCmd = 100;
      updateTargetsFromCommand();
      printCommandStatus();
      break;

    case 'z':
    case 'Z':
      speedPercentCmd -= CMD_STEP_PERCENT;
      if (speedPercentCmd < 0) speedPercentCmd = 0;
      updateTargetsFromCommand();
      printCommandStatus();
      break;

    case 'p':
    case 'P':
      printPIDStatus();
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
      Serial.printf("Unknown cmd: %c\n", c);
      break;
  }
}

void processSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    processImmediateCommand(c);
  }
}

// ==========================================
// LOG RUNTIME
// ==========================================
void printRuntimeStatus(unsigned long nowMs) {
  static unsigned long lastPrintMs = 0;

  if (nowMs - lastPrintMs < 200) {
    return;
  }
  lastPrintMs = nowMs;

  Serial.printf(
    "[RUN] cmd=%s | spd=%d%% | "
    "L: tgt=%.1f rpm=%.1f err=%.1f pwm=%d | "
    "R: tgt=%.1f rpm=%.1f err=%.1f pwm=%d\n",
    cmdToString(currentCmd),
    speedPercentCmd,
    targetLeftRPM,  M1.actualRPM, M1.error, M1.lastPWM,
    targetRightRPM, M2.actualRPM, M2.error, M2.lastPWM
  );
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  setupMotorHardware(M1, readEnc1);
  setupMotorHardware(M2, readEnc2);

  resetMotorControl(M1);
  resetMotorControl(M2);
  stopAllMotors();

  previousSampleMillis = millis();

  Serial.println();
  Serial.println("TELEOP PI + FEEDFORWARD READY");
  Serial.println("M1 = Left, M2 = Right");
  Serial.println("M2 inverted logically");
  printPIDStatus();
  printHelp();
  printCommandStatus();
}

// ==========================================
// LOOP
// ==========================================
void loop() {
  processSerial();

  unsigned long nowMs = millis();
  if (nowMs - previousSampleMillis < sampleTime) {
    return;
  }

  float dtSec = (nowMs - previousSampleMillis) / 1000.0f;
  previousSampleMillis = nowMs;

  updateBothRPM(dtSec);

  runPIDFF(M1, targetLeftRPM, dtSec);   // Left
  runPIDFF(M2, targetRightRPM, dtSec);  // Right

  printRuntimeStatus(nowMs);
}