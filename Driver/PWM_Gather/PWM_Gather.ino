// ==========================================
// CẤU HÌNH THÔNG SỐ (Giữ nguyên từ turn trước)
// ==========================================
const int GEAR_RATIO = 30; 
const float MAX_RPM = 333.0;
const int PULSES_PER_REV = 11 * GEAR_RATIO;

const int R_EN = 21, L_EN = 22, R_PWM = 23, L_PWM = 27;
const int ENC_A = 32, ENC_B = 33;
const int pwmFreq = 5000, pwmResolution = 8;

volatile long encoderCount = 0;
volatile unsigned long lastPulseTime = 0;
const unsigned long debounceDelay = 300; 

long previousCount = 0;
bool isTesting = false;
bool testCompleted = false;
unsigned long testStartTime = 0;
unsigned long lastStepTime = 0;
unsigned long holdStartTime = 0; // Thời điểm bắt đầu giữ 100%
int testPwm = 0;
const int sampleTime = 100; 
const int holdDuration = 2000; // 2 giây duy trì

void IRAM_ATTR readEncoder() {
  unsigned long currentMicros = micros();
  if (currentMicros - lastPulseTime > debounceDelay) {
    if (digitalRead(ENC_B) == HIGH) encoderCount++;
    else encoderCount--;
    lastPulseTime = currentMicros;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(R_EN, OUTPUT); pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, HIGH); digitalWrite(L_EN, HIGH);
  ledcAttach(R_PWM, pwmFreq, pwmResolution);
  ledcAttach(L_PWM, pwmFreq, pwmResolution);
  pinMode(ENC_A, INPUT_PULLUP); pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoder, RISING);
  Serial.println("Go 's' de bat dau lay mau (Ramp 0-100% + 2s Hold)...");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 's' || c == 'S') {
      ledcWrite(R_PWM, 0); ledcWrite(L_PWM, 0);
      delay(1000);
      isTesting = true; testCompleted = false;
      testPwm = 0; encoderCount = 0; previousCount = 0; holdStartTime = 0;
      Serial.println("Time(ms),PWM(%),RPM");
      testStartTime = millis(); lastStepTime = testStartTime;
    }
  }

  if (isTesting && !testCompleted) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastStepTime >= sampleTime) {
      noInterrupts();
      long currentCount = encoderCount;
      interrupts();
      long countDiff = currentCount - previousCount;
      previousCount = currentCount;
      float actualRPM = ((float)countDiff / PULSES_PER_REV) * (60000.0 / sampleTime);

      unsigned long timePassed = currentMillis - testStartTime;
      Serial.printf("%lu,%d,%.2f\n", timePassed, testPwm, actualRPM);

      // Logic tăng PWM hoặc Duy trì 100%
      if (testPwm < 100) {
        testPwm++;
      } else {
        // Nếu đã đạt 100, kiểm tra xem đã giữ đủ 2 giây chưa
        if (holdStartTime == 0) holdStartTime = currentMillis; 
        
        if (currentMillis - holdStartTime >= holdDuration) {
          testCompleted = true;
          isTesting = false;
          ledcWrite(R_PWM, 0); ledcWrite(L_PWM, 0);
          Serial.println("=== END OF DATA ===");
        }
      }

      if (!testCompleted) {
        int pwmValue = map(testPwm, 0, 100, 0, 255);
        ledcWrite(L_PWM, 0); ledcWrite(R_PWM, pwmValue);
      }
      lastStepTime = currentMillis;
    }
  }
}