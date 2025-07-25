#include <SoftwareSerial.h>

SoftwareSerial Serial1(12, 13);  // RX, TX

void setup() {
  Serial.begin(115200);    // Serial chính qua USB
  Serial1.begin(115200);     // Serial1 qua SoftwareSerial với tốc độ 9600 (để ổn định)
  Serial.println("ready");
}

void loop() {
  // Kiểm tra nếu có dữ liệu từ Serial1
  if (Serial1.available()) {
    String R = Serial1.readStringUntil('\n');
    Serial.println(R);  // Hiển thị chuỗi nhận được từ Serial1 lên Serial Monitor
  }

  // Kiểm tra nếu có dữ liệu từ Serial Monitor
  if (Serial.available()) {
    char C = Serial.read();  // Đọc ký tự từ cổng Serial

    // Gửi ký tự tương ứng tới Serial1 và in ra Serial Monitor
    if (C == 'F' || C == 'B' || C == 'R' || C == 'L' || 
        C == 'T' || C == 'U' || C == 'O' || C == 'V') {
      Serial1.println(C);
      Serial.println(C);
    }
  }
}
