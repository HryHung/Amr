// ==========================================
// CẤU HÌNH THÔNG SỐ CƠ BẢN
// ==========================================
const int GEAR_RATIO = 30; // 333 RPM version
const float MAX_RPM = 333.0;
const int PULSES_PER_REV = 11 * GEAR_RATIO; // 330 xung/vòng
const int sampleTime = 100; // Chu kỳ lấy mẫu PID: 100ms (Ts = 0.1s)
const float Ts = sampleTime / 1000.0;

const int pwmFreq = 5000;
const int pwmResolution = 8;

// ==========================================
// CẤU TRÚC ĐỐI TƯỢNG ĐỘNG CƠ (MOTOR STRUCT)
// ==========================================
struct Motor {
  // Chân kết nối
  int EN_R, EN_L, PWM_R, PWM_L, ENC_A, ENC_B;
  
  // Biến đếm xung
  volatile long count = 0;
  long prevCount = 0;
  volatile unsigned long lastPulseTime = 0;
  
  // Biến PID
  float Kp, Ki;
  float targetRPM = 0;
  float actualRPM = 0;
  float error = 0;
  float integral = 0;
};

// Khởi tạo Động cơ 1 (Thông số PID lấy từ phiên trước)
Motor M1 = {
  16, 17, 18, 19, 25, 26, // Các chân GPIO
  0, 0, 0,
  0.25, 2.10, // Kp, Ki của Động cơ 1
  0, 0, 0, 0
};

// Khởi tạo Động cơ 2 (Thông số PID bạn vừa cung cấp)
Motor M2 = {
  21, 27, 22, 23, 32, 33, // Các chân GPIO theo yêu cầu
  0, 0, 0,
  0.24, 2.15, // Kp, Ki của Động cơ 2
  0, 0, 0, 0
};

const unsigned long debounceDelay = 300; // Bộ lọc nhiễu 300us
unsigned long previousMillis = 0;

// ==========================================
// HÀM NGẮT ĐỘC LẬP CHO TỪNG ĐỘNG CƠ
// ==========================================
void IRAM_ATTR readEnc1() {
  unsigned long t = micros();
  if (t - M1.lastPulseTime > debounceDelay) {
    if (digitalRead(M1.ENC_B) == HIGH) M1.count++; else M1.count--;
    M1.lastPulseTime = t;
  }
}

void IRAM_ATTR readEnc2() {
  unsigned long t = micros();
  if (t - M2.lastPulseTime > debounceDelay) {
    if (digitalRead(M2.ENC_B) == HIGH) M2.count++; else M2.count--;
    M2.lastPulseTime = t;
  }
}

// ==========================================
// HÀM SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  Serial.println("HE THONG DIEU KHIEN 2 DONG CO (PI CONTROLLER)");
  Serial.println("Nhap van toc: RPM_1,RPM_2 (VD: 200,150 hoac 100,-200)");

  // Hàm cục bộ cấu hình phần cứng cho Motor
  auto setupMotor = [](Motor &m, void (*isr)()) {
    pinMode(m.EN_R, OUTPUT); pinMode(m.EN_L, OUTPUT);
    digitalWrite(m.EN_R, HIGH); digitalWrite(m.EN_L, HIGH); // Luôn Enable IBT-2
    
    ledcAttach(m.PWM_R, pwmFreq, pwmResolution);
    ledcAttach(m.PWM_L, pwmFreq, pwmResolution);
    ledcWrite(m.PWM_R, 0); ledcWrite(m.PWM_L, 0);

    pinMode(m.ENC_A, INPUT_PULLUP); pinMode(m.ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(m.ENC_A), isr, RISING);
  };

  setupMotor(M1, readEnc1);
  setupMotor(M2, readEnc2);
}

// ==========================================
// VÒNG LẶP CHÍNH
// ==========================================
void loop() {
  // 1. Nhận lệnh từ Serial (Định dạng: RPM1,RPM2)
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      float t1 = 0, t2 = 0;
      // Tách chuỗi lấy 2 giá trị vận tốc
      if (sscanf(input.c_str(), "%f,%f", &t1, &t2) == 2) {
        // Giới hạn vận tốc không vượt quá thiết kế cơ khí
        M1.targetRPM = constrain(t1, -MAX_RPM, MAX_RPM);
        M2.targetRPM = constrain(t2, -MAX_RPM, MAX_RPM);
        Serial.printf(">> SET M1: %.1f RPM | M2: %.1f RPM\n", M1.targetRPM, M2.targetRPM);
      } else {
        Serial.println("Loi cu phap! Vui long nhap dung: RPM1,RPM2");
      }
    }
  }

  // 2. Tính toán PID mỗi 100ms
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= sampleTime) {
    
    // Hàm cục bộ xử lý PID cho từng motor
    auto computePID = [](Motor &m, int id) {
      // Đọc xung an toàn
      noInterrupts();
      long currentCount = m.count;
      interrupts();
      
      long countDiff = currentCount - m.prevCount;
      m.prevCount = currentCount;

      // Tính RPM thực tế
      m.actualRPM = ((float)countDiff / PULSES_PER_REV) * (60.0 / Ts);

      // Thuật toán PI (Vận tốc)
      m.error = m.targetRPM - m.actualRPM;
      m.integral += m.error * Ts; // Tích lũy sai số theo thời gian

      // ANTI-WINDUP: Giới hạn khâu Integral để không bị tràn PWM (Max = 255)
      // Giới hạn an toàn khoảng 255 / Ki để bù đắp đủ 100% công suất
      float maxIntegral = 255.0 / m.Ki;
      m.integral = constrain(m.integral, -maxIntegral, maxIntegral);

      // Nếu muốn dừng hẳn (Target = 0), xóa bộ nhớ Integral
      if (m.targetRPM == 0) m.integral = 0; 

      // Tính Output
      float output = (m.Kp * m.error) + (m.Ki * m.integral);
      int finalPWM = constrain(round(output), -255, 255);

      // DEAD-BAND COMPENSATION (Bù Vùng Chết 4%)
      // Động cơ cần khoảng 10 PWM mới bắt đầu nhích
      if (finalPWM > 0 && finalPWM < 10) finalPWM = 10;
      if (finalPWM < 0 && finalPWM > -10) finalPWM = -10;
      if (m.targetRPM == 0) finalPWM = 0; // Tắt hoàn toàn

      // Xuất tín hiệu ra IBT-2
      if (finalPWM >= 0) {
        ledcWrite(m.PWM_L, 0); ledcWrite(m.PWM_R, finalPWM);
      } else {
        ledcWrite(m.PWM_R, 0); ledcWrite(m.PWM_L, abs(finalPWM));
      }

      // In log theo dõi
      Serial.printf("M%d [Target: %4.0f | Act: %6.1f | PWM: %4d]  ", id, m.targetRPM, m.actualRPM, finalPWM);
    };

    // Chạy PID cho cả 2 motor
    computePID(M1, 1);
    computePID(M2, 2);
    Serial.println(); // Xuống dòng cho đẹp

    previousMillis = currentMillis;
  }
}