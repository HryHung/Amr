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

// Sweep mode
const unsigned long PRE_DELAY_MS = 1000;
const unsigned long PEAK_HOLD_MS = 2000;
const int PWM_STEP_PERCENT = 1;

// Script mode
const unsigned long SCRIPT_STOP_MS = 1000;
const unsigned long SCRIPT_MOVE_MS = 2500;
const float SCRIPT_RPM = 120.0f;

// Mode 3 input timeout
const unsigned long RPM_INPUT_TIMEOUT_MS = 300;

// Feedforward / safety
const float TARGET_ZERO_EPS_RPM = 1.0f;
const float FF_MAX_FRACTION = 0.85f;      // feedforward không chiếm toàn bộ PWM
const float PI_INTEGRAL_FRACTION = 0.60f; // giới hạn phần tích phân

// ==========================================
// MODE MASTER
// ==========================================
enum MasterMode {
  MODE_IDLE = 0,
  MODE_SWEEP_OPEN,
  MODE_SWEEP_PID,
  MODE_RPM_HOLD,
  MODE_SCRIPT
};

enum SweepState {
  SWEEP_IDLE = 0,
  SWEEP_PRE_DELAY,
  SWEEP_RAMP_UP,
  SWEEP_HOLD_PEAK,
  SWEEP_DONE
};

enum ScriptState {
  SCRIPT_IDLE = 0,
  SCRIPT_STOP_0,
  SCRIPT_FORWARD,
  SCRIPT_STOP_1,
  SCRIPT_BACKWARD,
  SCRIPT_STOP_2,
  SCRIPT_LEFT,
  SCRIPT_STOP_3,
  SCRIPT_RIGHT,
  SCRIPT_STOP_4,
  SCRIPT_DONE
};

// ==========================================
// CẤU TRÚC MOTOR
// ==========================================
struct Motor {
  int EN_R, EN_L, PWM_R, PWM_L, ENC_A, ENC_B;

  volatile long count;
  long prevCount;
  volatile unsigned long lastPulseTime;

  // PID
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
MasterMode masterMode = MODE_IDLE;
SweepState sweepState = SWEEP_IDLE;
ScriptState scriptState = SCRIPT_IDLE;

unsigned long modeStartMillis = 0;
unsigned long stateStartMillis = 0;
unsigned long previousSampleMillis = 0;

int pwmPercentCmd = 0;
float rpmHoldCmd = 0.0f;

bool waitingForRPMInput = false;
String rpmInputBuffer = "";
unsigned long rpmInputLastCharMillis = 0;

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

bool isRPMChar(char c) {
  return ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.');
}

const char* modeToString(MasterMode m) {
  switch (m) {
    case MODE_IDLE:       return "IDLE";
    case MODE_SWEEP_OPEN: return "SWEEP_OPEN";
    case MODE_SWEEP_PID:  return "SWEEP_PID";
    case MODE_RPM_HOLD:   return "RPM_HOLD";
    case MODE_SCRIPT:     return "SCRIPT";
    default:              return "UNKNOWN";
  }
}

const char* scriptStateToString(ScriptState s) {
  switch (s) {
    case SCRIPT_STOP_0:   return "STOP_0";
    case SCRIPT_FORWARD:  return "FORWARD";
    case SCRIPT_STOP_1:   return "STOP_1";
    case SCRIPT_BACKWARD: return "BACKWARD";
    case SCRIPT_STOP_2:   return "STOP_2";
    case SCRIPT_LEFT:     return "LEFT";
    case SCRIPT_STOP_3:   return "STOP_3";
    case SCRIPT_RIGHT:    return "RIGHT";
    case SCRIPT_STOP_4:   return "STOP_4";
    case SCRIPT_DONE:     return "DONE";
    default:              return "IDLE";
  }
}

void printHelp() {
  Serial.println();
  Serial.println("============= MASTER MOTOR DEBUG =============");
  Serial.println("1 : gather data khong PID");
  Serial.println("2 : gather data co PID + feedforward");
  Serial.println("3 : nhap RPM roi Send (vd: 200 hoac -150)");
  Serial.println("4 : chay kich ban thang/lui/trai/phai");
  Serial.println("x : stop/abort");
  Serial.println("p : in PID + FF hien tai");
  Serial.println("h : help");
  Serial.println("----------------------------------------------");
  Serial.println("Mode 3:");
  Serial.println("  bam 3 -> nhap so RPM -> Send");
  Serial.println("  hoat dong ca voi No line ending");
  Serial.println("==============================================");
  Serial.println();
}

void printPIDStatus() {
  Serial.printf("M1 Left  PID: Kp=%.4f Ki=%.4f Kd=%.4f | FF: Kff=%.4f Off=%.2f Tau=%.3f\n",
                M1.Kp, M1.Ki, M1.Kd, M1.Kff, M1.ffOffset, M1.ffTau);
  Serial.printf("M2 Right PID: Kp=%.4f Ki=%.4f Kd=%.4f | FF: Kff=%.4f Off=%.2f Tau=%.3f\n",
                M2.Kp, M2.Ki, M2.Kd, M2.Kff, M2.ffOffset, M2.ffTau);
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

void resetAllRuntime() {
  resetMotorControl(M1);
  resetMotorControl(M2);
  stopAllMotors();

  pwmPercentCmd = 0;
  rpmHoldCmd = 0.0f;
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
// PID + FEEDFORWARD
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

void abortAllModes() {
  stopAllMotors();
  masterMode = MODE_IDLE;
  sweepState = SWEEP_IDLE;
  scriptState = SCRIPT_IDLE;
  waitingForRPMInput = false;
  rpmInputBuffer = "";
  Serial.println("EVENT,ABORT");
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
// START MODES
// ==========================================
void startSweepOpen() {
  resetAllRuntime();
  masterMode = MODE_SWEEP_OPEN;
  sweepState = SWEEP_PRE_DELAY;

  modeStartMillis = millis();
  stateStartMillis = modeStartMillis;
  previousSampleMillis = modeStartMillis;

  Serial.println();
  Serial.println("EVENT,START,SWEEP_OPEN");
  Serial.println("Time_ms,PWM_percent,RPM_Motor_R,RPM_Motor_L");
}

void startSweepPID() {
  resetAllRuntime();
  masterMode = MODE_SWEEP_PID;
  sweepState = SWEEP_PRE_DELAY;

  modeStartMillis = millis();
  stateStartMillis = modeStartMillis;
  previousSampleMillis = modeStartMillis;

  Serial.println();
  Serial.println("EVENT,START,SWEEP_PID_FF");
  Serial.println("Time_ms,Target_percent,TargetRPM,RPM_Motor_R,RPM_Motor_L,PWM_R,PWM_L");
}

void startRPMHold(float rpmCmd) {
  resetAllRuntime();
  masterMode = MODE_RPM_HOLD;

  rpmHoldCmd = clampf(rpmCmd, -MAX_RPM, MAX_RPM);

  modeStartMillis = millis();
  stateStartMillis = modeStartMillis;
  previousSampleMillis = modeStartMillis;

  Serial.println();
  Serial.printf("EVENT,START,RPM_HOLD_FF,Cmd=%.3f\n", rpmHoldCmd);
  Serial.println("Time_ms,TargetRPM,RPM_Motor_R,RPM_Motor_L,PWM_R,PWM_L,Err_R,Err_L");
}

void startScript() {
  resetAllRuntime();
  masterMode = MODE_SCRIPT;
  scriptState = SCRIPT_STOP_0;

  modeStartMillis = millis();
  stateStartMillis = modeStartMillis;
  previousSampleMillis = modeStartMillis;

  Serial.println();
  Serial.println("EVENT,START,SCRIPT_FF");
  Serial.println("Time_ms,Phase,TargetRPM_R,TargetRPM_L,RPM_Motor_R,RPM_Motor_L,PWM_R,PWM_L");
}

// ==========================================
// MODE 1: SWEEP OPEN LOOP
// ==========================================
void updateSweepOpen(unsigned long nowMs) {
  unsigned long elapsedState = nowMs - stateStartMillis;

  if (sweepState == SWEEP_PRE_DELAY) {
    pwmPercentCmd = 0;
    applyMotorPWM(M1, 0);
    applyMotorPWM(M2, 0);

    if (elapsedState >= PRE_DELAY_MS) {
      sweepState = SWEEP_RAMP_UP;
      stateStartMillis = nowMs;
      pwmPercentCmd = 0;
      Serial.println("EVENT,STATE,RAMP_UP");
    }
  }
  else if (sweepState == SWEEP_RAMP_UP) {
    int pwmCmd = percentToPwm(pwmPercentCmd);
    applyMotorPWM(M1, pwmCmd);
    applyMotorPWM(M2, pwmCmd);

    if (pwmPercentCmd < 100) {
      pwmPercentCmd += PWM_STEP_PERCENT;
      if (pwmPercentCmd > 100) pwmPercentCmd = 100;
    } else {
      sweepState = SWEEP_HOLD_PEAK;
      stateStartMillis = nowMs;
      Serial.println("EVENT,STATE,HOLD_PEAK");
    }
  }
  else if (sweepState == SWEEP_HOLD_PEAK) {
    pwmPercentCmd = 100;
    int pwmCmd = percentToPwm(100);
    applyMotorPWM(M1, pwmCmd);
    applyMotorPWM(M2, pwmCmd);

    if (elapsedState >= PEAK_HOLD_MS) {
      sweepState = SWEEP_DONE;
      stopAllMotors();
      Serial.println("EVENT,STATE,DONE");
      Serial.println("EVENT,END,SWEEP_OPEN");
      masterMode = MODE_IDLE;
    }
  }

  unsigned long t = nowMs - modeStartMillis;
  Serial.printf("%lu,%d,%.3f,%.3f\n", t, pwmPercentCmd, M2.actualRPM, M1.actualRPM);
}

// ==========================================
// MODE 2: SWEEP PID + FF
// ==========================================
void updateSweepPID(unsigned long nowMs, float dtSec) {
  unsigned long elapsedState = nowMs - stateStartMillis;
  float targetPercent = 0.0f;
  float targetRPM = 0.0f;

  if (sweepState == SWEEP_PRE_DELAY) {
    targetPercent = 0.0f;
    targetRPM = 0.0f;

    runPIDFF(M1, targetRPM, dtSec);
    runPIDFF(M2, targetRPM, dtSec);

    if (elapsedState >= PRE_DELAY_MS) {
      sweepState = SWEEP_RAMP_UP;
      stateStartMillis = nowMs;
      pwmPercentCmd = 0;
      Serial.println("EVENT,STATE,RAMP_UP");
    }
  }
  else if (sweepState == SWEEP_RAMP_UP) {
    targetPercent = (float)pwmPercentCmd;
    targetRPM = (targetPercent / 100.0f) * MAX_RPM;

    runPIDFF(M1, targetRPM, dtSec);
    runPIDFF(M2, targetRPM, dtSec);

    if (pwmPercentCmd < 100) {
      pwmPercentCmd += PWM_STEP_PERCENT;
      if (pwmPercentCmd > 100) pwmPercentCmd = 100;
    } else {
      sweepState = SWEEP_HOLD_PEAK;
      stateStartMillis = nowMs;
      Serial.println("EVENT,STATE,HOLD_PEAK");
    }
  }
  else if (sweepState == SWEEP_HOLD_PEAK) {
    targetPercent = 100.0f;
    targetRPM = MAX_RPM;

    runPIDFF(M1, targetRPM, dtSec);
    runPIDFF(M2, targetRPM, dtSec);

    if (elapsedState >= PEAK_HOLD_MS) {
      sweepState = SWEEP_DONE;
      stopAllMotors();
      Serial.println("EVENT,STATE,DONE");
      Serial.println("EVENT,END,SWEEP_PID_FF");
      masterMode = MODE_IDLE;
    }
  }

  unsigned long t = nowMs - modeStartMillis;
  Serial.printf("%lu,%.1f,%.3f,%.3f,%.3f,%d,%d\n",
                t, targetPercent, targetRPM, M2.actualRPM, M1.actualRPM, M2.lastPWM, M1.lastPWM);
}

// ==========================================
// MODE 3: HOLD RPM + FF
// ==========================================
void updateRPMHold(unsigned long nowMs, float dtSec) {
  runPIDFF(M1, rpmHoldCmd, dtSec);
  runPIDFF(M2, rpmHoldCmd, dtSec);

  unsigned long t = nowMs - modeStartMillis;
  Serial.printf("%lu,%.3f,%.3f,%.3f,%d,%d,%.3f,%.3f\n",
                t, rpmHoldCmd, M2.actualRPM, M1.actualRPM, M2.lastPWM, M1.lastPWM, M2.error, M1.error);
}

// ==========================================
// MODE 4: SCRIPT + FF
// ==========================================
void updateScript(unsigned long nowMs, float dtSec) {
  unsigned long elapsedState = nowMs - stateStartMillis;

  float targetR = 0.0f;
  float targetL = 0.0f;

  switch (scriptState) {
    case SCRIPT_STOP_0:
      targetR = 0.0f;
      targetL = 0.0f;
      if (elapsedState >= SCRIPT_STOP_MS) {
        scriptState = SCRIPT_FORWARD;
        stateStartMillis = nowMs;
      }
      break;

    case SCRIPT_FORWARD:
      targetR = +SCRIPT_RPM;
      targetL = +SCRIPT_RPM;
      if (elapsedState >= SCRIPT_MOVE_MS) {
        scriptState = SCRIPT_STOP_1;
        stateStartMillis = nowMs;
      }
      break;

    case SCRIPT_STOP_1:
      targetR = 0.0f;
      targetL = 0.0f;
      if (elapsedState >= SCRIPT_STOP_MS) {
        scriptState = SCRIPT_BACKWARD;
        stateStartMillis = nowMs;
      }
      break;

    case SCRIPT_BACKWARD:
      targetR = -SCRIPT_RPM;
      targetL = -SCRIPT_RPM;
      if (elapsedState >= SCRIPT_MOVE_MS) {
        scriptState = SCRIPT_STOP_2;
        stateStartMillis = nowMs;
      }
      break;

    case SCRIPT_STOP_2:
      targetR = 0.0f;
      targetL = 0.0f;
      if (elapsedState >= SCRIPT_STOP_MS) {
        scriptState = SCRIPT_LEFT;
        stateStartMillis = nowMs;
      }
      break;

    case SCRIPT_LEFT:
      targetR = +SCRIPT_RPM;
      targetL = -SCRIPT_RPM;
      if (elapsedState >= SCRIPT_MOVE_MS) {
        scriptState = SCRIPT_STOP_3;
        stateStartMillis = nowMs;
      }
      break;

    case SCRIPT_STOP_3:
      targetR = 0.0f;
      targetL = 0.0f;
      if (elapsedState >= SCRIPT_STOP_MS) {
        scriptState = SCRIPT_RIGHT;
        stateStartMillis = nowMs;
      }
      break;

    case SCRIPT_RIGHT:
      targetR = -SCRIPT_RPM;
      targetL = +SCRIPT_RPM;
      if (elapsedState >= SCRIPT_MOVE_MS) {
        scriptState = SCRIPT_STOP_4;
        stateStartMillis = nowMs;
      }
      break;

    case SCRIPT_STOP_4:
      targetR = 0.0f;
      targetL = 0.0f;
      if (elapsedState >= SCRIPT_STOP_MS) {
        scriptState = SCRIPT_DONE;
        stopAllMotors();
        Serial.println("EVENT,END,SCRIPT_FF");
        masterMode = MODE_IDLE;
      }
      break;

    default:
      targetR = 0.0f;
      targetL = 0.0f;
      break;
  }

  if (masterMode == MODE_SCRIPT) {
    runPIDFF(M2, targetR, dtSec); // Right
    runPIDFF(M1, targetL, dtSec); // Left

    unsigned long t = nowMs - modeStartMillis;
    Serial.printf("%lu,%s,%.3f,%.3f,%.3f,%.3f,%d,%d\n",
                  t, scriptStateToString(scriptState),
                  targetR, targetL, M2.actualRPM, M1.actualRPM, M2.lastPWM, M1.lastPWM);
  }
}

// ==========================================
// SERIAL
// ==========================================
void processImmediateCommand(char c) {
  switch (c) {
    case '1':
      startSweepOpen();
      break;

    case '2':
      startSweepPID();
      break;

    case '3':
      waitingForRPMInput = true;
      rpmInputBuffer = "";
      rpmInputLastCharMillis = millis();
      Serial.println("Nhap RPM roi Send. Vi du: 200 hoac -150");
      Serial.println("Khong can line ending, tu dong nhan sau 300 ms im lang.");
      break;

    case '4':
      startScript();
      break;

    case 'x':
    case 'X':
      abortAllModes();
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

void finalizeRPMInput() {
  if (rpmInputBuffer.length() == 0) {
    waitingForRPMInput = false;
    return;
  }

  float rpm = rpmInputBuffer.toFloat();
  waitingForRPMInput = false;
  rpmInputBuffer = "";
  startRPMHold(rpm);
}

void processSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (waitingForRPMInput) {
      if (c == '\n' || c == '\r' || c == ' ') {
        finalizeRPMInput();
      } else if (isRPMChar(c)) {
        rpmInputBuffer += c;
        rpmInputLastCharMillis = millis();
      } else {
        if (rpmInputBuffer.length() > 0) {
          finalizeRPMInput();
        }
        processImmediateCommand(c);
      }
    } else {
      processImmediateCommand(c);
    }
  }
}

void serviceRPMInputTimeout() {
  if (waitingForRPMInput && rpmInputBuffer.length() > 0) {
    if (millis() - rpmInputLastCharMillis >= RPM_INPUT_TIMEOUT_MS) {
      finalizeRPMInput();
    }
  }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  setupMotorHardware(M1, readEnc1);
  setupMotorHardware(M2, readEnc2);

  stopAllMotors();

  Serial.println();
  Serial.println("MASTER MOTOR DEBUG READY");
  Serial.println("M1 = Left, M2 = Right");
  Serial.println("M2 inverted logically");
  printPIDStatus();
  printHelp();
}

// ==========================================
// LOOP
// ==========================================
void loop() {
  processSerial();
  serviceRPMInputTimeout();

  if (masterMode == MODE_IDLE) {
    return;
  }

  unsigned long nowMs = millis();
  if (nowMs - previousSampleMillis < sampleTime) {
    return;
  }

  float dtSec = (nowMs - previousSampleMillis) / 1000.0f;
  previousSampleMillis = nowMs;

  updateBothRPM(dtSec);

  switch (masterMode) {
    case MODE_SWEEP_OPEN:
      updateSweepOpen(nowMs);
      break;

    case MODE_SWEEP_PID:
      updateSweepPID(nowMs, dtSec);
      break;

    case MODE_RPM_HOLD:
      updateRPMHold(nowMs, dtSec);
      break;

    case MODE_SCRIPT:
      updateScript(nowMs, dtSec);
      break;

    default:
      break;
  }
}