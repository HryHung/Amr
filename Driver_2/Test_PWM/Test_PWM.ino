// ==========================================
// 1. CẤU HÌNH THÔNG SỐ ĐỘNG CƠ 
// ==========================================
const int GEAR_RATIO = 30;              
const float MAX_RPM = 333.0;            
const int PULSES_PER_REV = 11 * GEAR_RATIO; 

// ==========================================
// 2. KHAI BÁO CHÂN KẾT NỐI
// ==========================================
const int R_EN = 21;
const int L_EN = 22;
const int R_PWM = 23;
const int L_PWM = 27;
const int ENC_A = 32;
const int ENC_B = 33;

const int pwmFreq = 5000;       
const int pwmResolution = 8;    

// ==========================================
// THÊM MỚI: CÁC BIẾN LỌC NHIỄU (DEBOUNCE)
// ==========================================
volatile long encoderCount = 0; 
volatile unsigned long lastPulseTime = 0; // Thời gian nhận xung cuối cùng
const unsigned long debounceDelay = 300;  // Ngưỡng lọc nhiễu: 300 micro-giây

long previousCount = 0;         
unsigned long previousMillis = 0; 
const int sampleTime = 100;     

int currentPwmPercent = 0;      
float targetRPM = 0;            
float actualRPM = 0;            

// ==========================================
// 3. HÀM NGẮT - ĐÃ TÍCH HỢP BỘ LỌC NHIỄU KHỦNG
// ==========================================
void IRAM_ATTR readEncoder() {
  unsigned long currentMicros = micros(); // Đọc đồng hồ micro-giây của ESP32
  
  // Chỉ chấp nhận đếm xung nếu khoảng cách thời gian lớn hơn 300us
  if (currentMicros - lastPulseTime > debounceDelay) {
    if (digitalRead(ENC_B) == HIGH) { // Đã đảo HIGH để fix chiều quay
      encoderCount++; 
    } else {
      encoderCount--; 
    }
    lastPulseTime = currentMicros; // Lưu lại thời điểm nhận xung thật
  }
}

// ==========================================
// 4. HÀM SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  Serial.println("HE THONG TEST DONG CO JGB37 - FILTERED VERSION");
  Serial.println("Nhap % PWM tu -100 den 100 vao o gui Serial de dieu khien:");

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
}

// ==========================================
// 5. VÒNG LẶP CHÍNH
// ==========================================
void loop() {
  // --- BƯỚC 1: NHẬN LỆNH PWM TỪ SERIAL ---
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() > 0) {
      currentPwmPercent = input.toInt();
      currentPwmPercent = constrain(currentPwmPercent, -100, 100);
      
      targetRPM = ((float)currentPwmPercent / 100.0) * MAX_RPM;
      int pwmValue = map(abs(currentPwmPercent), 0, 100, 0, 255);
      
      if (currentPwmPercent > 0) { 
        ledcWrite(L_PWM, 0);
        ledcWrite(R_PWM, pwmValue);
      } else if (currentPwmPercent < 0) { 
        ledcWrite(R_PWM, 0);
        ledcWrite(L_PWM, pwmValue);
      } else { 
        ledcWrite(R_PWM, 0);
        ledcWrite(L_PWM, 0);
      }
    }
  }

  // --- BƯỚC 2: TÍNH RPM MỖI 100ms ---
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= sampleTime) {
    noInterrupts();
    long currentCount = encoderCount;
    interrupts();

    long countDiff = currentCount - previousCount;
    previousCount = currentCount;

    actualRPM = ((float)countDiff / PULSES_PER_REV) * (60000.0 / sampleTime);

    Serial.printf("PWM: %4d %% | Target: %7.2f RPM | Actual: %7.2f RPM\n", 
                  currentPwmPercent, targetRPM, actualRPM);

    previousMillis = currentMillis;
  }
}