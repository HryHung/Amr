// ==========================================
// THÔNG SỐ ĐỘNG CƠ VÀ CHÂN KẾT NỐI
// ==========================================
const int GEAR_RATIO = 30; // Chú ý: Đổi thành 270 nếu dùng loại 37RPM
const float MAX_RPM = 333.0;
const int PULSES_PER_REV = 11 * GEAR_RATIO;

const int R_EN = 21;
const int L_EN = 22;
const int R_PWM = 23;
const int L_PWM = 27;
const int ENC_A = 32;
const int ENC_B = 33;

const int pwmFreq = 5000;
const int pwmResolution = 8;

// Biến Encoder và Lọc nhiễu
volatile long encoderCount = 0;
volatile unsigned long lastPulseTime = 0;
const unsigned long debounceDelay = 300; // Bộ lọc 300us

long previousCount = 0;

// Biến cho quá trình lấy mẫu (System ID)
bool isTesting = false;
bool testCompleted = false;
unsigned long testStartTime = 0;
unsigned long lastStepTime = 0;
int testPwm = 0;
const int sampleTime = 100; // Lấy mẫu mỗi 100ms (0.1s)

// ==========================================
// HÀM NGẮT ENCODER
// ==========================================
void IRAM_ATTR readEncoder() {
  unsigned long currentMicros = micros();
  if (currentMicros - lastPulseTime > debounceDelay) {
    if (digitalRead(ENC_B) == HIGH) {
      encoderCount++;
    } else {
      encoderCount--;
    }
    lastPulseTime = currentMicros;
  }
}

// ==========================================
// HÀM SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  ledcAttach(R_PWM, pwmFreq, pwmResolution);
  ledcAttach(L_PWM, pwmFreq, pwmResolution);
  ledcWrite(R_PWM, 0);
  ledcWrite(L_PWM, 0);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoder, RISING);

  Serial.println("HE THONG LAY MAU DATA CHO MATLAB (SYSTEM IDENTIFICATION)");
  Serial.println("Go ky tu 's' roi an Enter de bat dau lay mau...");
}

// ==========================================
// VÒNG LẶP CHÍNH
// ==========================================
void loop() {
  // 1. Chờ lệnh 's' từ Serial
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 's' || c == 'S') {
      ledcWrite(R_PWM, 0);
      ledcWrite(L_PWM, 0);
      Serial.println(">> Da nhan lenh. Dung dong co 1 giay...");
      delay(1000); // Dừng hẳn 1s theo yêu cầu

      // Khởi tạo các biến để bắt đầu test
      isTesting = true;
      testCompleted = false;
      testPwm = 0;
      encoderCount = 0;
      previousCount = 0;
      
      Serial.println("Time(ms),PWM(%),RPM"); // In Header cho bảng dữ liệu
      testStartTime = millis();
      lastStepTime = testStartTime;
    }
  }

  // 2. Chạy kịch bản tăng PWM từ 0 -> 100
  if (isTesting && !testCompleted) {
    unsigned long currentMillis = millis();
    
    // Cứ mỗi 100ms (0.1s) thì tăng 1 step và ghi data
    if (currentMillis - lastStepTime >= sampleTime) {
      
      // Tính RPM của 100ms vừa qua
      noInterrupts();
      long currentCount = encoderCount;
      interrupts();
      long countDiff = currentCount - previousCount;
      previousCount = currentCount;
      
      float actualRPM = ((float)countDiff / PULSES_PER_REV) * (60000.0 / sampleTime);

      // In ra định dạng CSV: Thời gian, %PWM, Tốc độ RPM
      unsigned long timePassed = currentMillis - testStartTime;
      Serial.printf("%lu,%d,%.2f\n", timePassed, testPwm, actualRPM);

      // Tăng PWM cho step tiếp theo
      testPwm++;
      
      if (testPwm > 100) {
        // Đã đạt 100%, kết thúc test
        testCompleted = true;
        isTesting = false;
        ledcWrite(R_PWM, 0); // Ngắt động cơ an toàn
        ledcWrite(L_PWM, 0);
        Serial.println("=== END OF DATA ===");
        Serial.println("Copy khoi du lieu tren (tu dong Time... den END...) sang file .csv hoac MATLAB.");
      } else {
        // Xuất PWM ra IBT-2
        int pwmValue = map(testPwm, 0, 100, 0, 255);
        ledcWrite(L_PWM, 0);
        ledcWrite(R_PWM, pwmValue);
      }
      
      lastStepTime = currentMillis;
    }
  }
}