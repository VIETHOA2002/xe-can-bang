/* Source code read sensor MPU6050: Copyright (C) 2012 Kristian Lauszus, TKJ Electronics */

#include <Wire.h> //khai báo thư viện hỗ trợ giao tiếp I2C 
#include "Kalman.h"  // Source: https://github.com/TKJElectronics/KalmanFilter
#include <avr/io.h>
#include <avr/interrupt.h>
#include<MPU6050.h>
#include <avr/wdt.h> // Thư viện để sử dụng Watchdog Timer

Kalman kalmanX; //Kalman filter define: Kalman.getAngle(pitch,gyrotate,dt);

#define factortheta PI/180  // The theta setpoint value change ever 7ms if control
#define factorphi PI/180 //Hằng số để chuyển đổi từ độ sang radian 
int inChar; //
uint32_t timerloop, timerold; //Biến lưu thời gian cho vòng lặp và xử lý thời gian mẫu
char inByte; //Dữ liệu nhận từ Bluetooth

//================================================================================================//
//Motor control Pin//
int leftpwm = 46; // Chân PWN của động cơ
int leftmotor1 = 34; // Chân điều khiển chiều quay động cơ
int leftmotor2 = 36;
int righpwm = 44;
int righmotor1 = 40;
int righmotor2 = 42;
int STBY = 38; // chân 
//============================================
//=============================================
volatile long leftencoder;  // lưu biến nằm ngoài sự kiểm soát của chương trình đang chạy
volatile long righencoder;
volatile long leftencoder_1;  // lưu biến nằm ngoài sự kiểm soát của chương trình đang chạy
volatile long righencoder_1;
volatile long leftencoder_2;  // lưu biến nằm ngoài sự kiểm soát của chương trình đang chạy
volatile long righencoder_2;

int leftencoder_a = 2;
int leftencoder_b = 5;
int righencoder_a = 3;
int righencoder_b = 7;

//unsigned long lastTime;


float mpudata;  //Save psi angle ( X axis)
float Acx, AcY, AcZ; // giá trị con quay hồi chuyển trục x, y, z
float Gxro; // gia tốc góc theo trục y
float Gxrate;
float Gxangle;
float pitch, offset, GHPWM;
float R;


uint32_t timer; // tính toán thời gian đọc dữ liệu từ MPU
uint8_t i2cData[14];  // chuỗi lưu trữ dữ liệu 14 byte đọc giá trị từ cảm biến MPU

//LQR data//
long PWML, PWMR;               // PWM output for H-Brigde
float k1, k2, k3, k4, k5, k6;  // The factor of K maxtrix
bool falldown;                 // Run = true; Stop  = false; biến điều khiển xe

float theta, psi, phi, theta_1, phi_1;
float thetadot, psidot, phidot; // 3 biến cập nhập tốc độ thay đổi của nó
float thetaold, psiold, phiold; // 3 biến cập nhập giá trị của nó trước 1 chu kỳ

float leftvolt;  //output volt left motor in LQR
float righvolt;  //output volt right motor in lQR

float addtheta;  //Save setpoint value giá trị đặt trước để mô hình xe bám theo
float addphi;    //Save setpoint value

int ForwardBack;  // 1 -> Forward;   -1 -> Back;      0 -> Stop And Balancing
int LeftRight;    // 1 -> Turnleft;  -1 -> TurnRight  0 -> Stop And Balancing

unsigned long lastSendTime = 0; // Thời điểm gửi dữ liệu cuối cùng
const unsigned long sendInterval = 100; // Khoảng thời gian giữa các lần gửi (milliseconds)

//=========================================== Start Setup =========================================//
void setup() {
  //======================== Khai bao ngat ===================================//
  TCCR5B = TCCR5B & B11111000 | B00000001;  //Pin 44 & Pin 46  https://arduinoinfo.mywikis.net/wiki/Arduino-PWM-Frequency  set PWM FREQUENCY 31kHz
  // định nghĩa tín hiệu PWN set tần 31kHz để điều khiển động cơ êm hơn 
  Serial.begin(115200); // 
  Serial1.begin(115200); // giao tiếp kết nối bluetooth để điều khiển mô hình từ xa
  k1 = 3.2;//1.95;//    bánh xe chạy nhanh
  k2 = 0.55;//; //    tốc độ bánh xe
  k3 = 37;//45; //k3*psi            góc
  k4 = 0.7;//0.8;//k4*psidot       tốc độ góc
  k5 = 6;    //10;  //k5*phi
  k6 = 0.8;    //k6*phidot
  offset = 0;
  GHPWM = 330;
  ForwardBack = 0;
  LeftRight = 0;
  addphi = 0;
  addtheta = 0;
  pinMode(leftpwm, OUTPUT);
  pinMode(righpwm, OUTPUT);
  pinMode(leftmotor1, OUTPUT);
  pinMode(leftmotor2, OUTPUT);
  pinMode(righmotor1, OUTPUT);
  pinMode(righmotor2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  
  
  pinMode(leftencoder_a, INPUT_PULLUP);
  pinMode(leftencoder_b, INPUT_PULLUP);
  pinMode(righencoder_a, INPUT_PULLUP);
  pinMode(righencoder_b, INPUT_PULLUP);

  attachInterrupt(0, left_isr, RISING); // pin 2 
  attachInterrupt(1, righ_isr, RISING); // pin 3
// DATA MPU6050 //
  Wire.begin();
  i2cData[0] = 7;     
  i2cData[1] = 0x00;  
  i2cData[2] = 0x00;  
  i2cData[3] = 0x00;  
  while (i2cWrite(0x19, i2cData, 4, false));  
  while (i2cWrite(0x6B, 0x01, true));  
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { 
    Serial.print(F("Error reading sensor"));
    while (1);
  }
  delay(100);   
  while (i2cRead(0x3B, i2cData, 6));
  AcY = -((i2cData[2] << 8) | i2cData[3]);
  AcZ = -((i2cData[4] << 8) | i2cData[5]);
  mpudata = (atan2(AcY, AcZ)) * RAD_TO_DEG;
  kalmanX.setAngle(mpudata);  // Set starting angl
  timer = micros();
}
void loop() {
  READ_MPU();
  Bluetooth();
  if ((micros() - timerloop) > 4000) {  //thời gian lấy mẫu
    psi = (Acx - 1.5 + offset) * DEG_TO_RAD;     //
    theta = gettheta(leftencoder, righencoder)*DEG_TO_RAD;
    theta_1 = gettheta(leftencoder_1, righencoder_1)*DEG_TO_RAD;
    phi = getphi(leftencoder_1, righencoder_1)*DEG_TO_RAD;
    phi_1 = getphi(leftencoder_2, righencoder_2)*DEG_TO_RAD;
    float dt = (float)(micros() - timerloop) / 1000000.0;
    timerloop = micros();
    psidot = (psi - psiold) / dt;
    thetadot = (theta - thetaold) / dt; // cập nhập giá mới
    phidot = (phi - phiold) / dt;
    psiold = psi;
    thetaold = theta;
    phiold = phi;
    addtheta = addtheta + ForwardBack*factortheta; // khi có tín hiệu điều khiển từ Uno biến FowardBack set lên 1
    addphi = 45/180;
    getlqr(theta + addtheta, thetadot, psi, psidot, phi+addphi, phidot);
    motorcontrol(PWML, PWMR, psi*RAD_TO_DEG, falldown);
  }
  Serial.println(leftencoder);
  Serial.println(psi);
  unsigned long currentTime = millis();
   if (currentTime - lastSendTime >= sendInterval) {
    String S = String(psi*RAD_TO_DEG) + "," + String(theta_1*RAD_TO_DEG) + "," + String(phi_1*RAD_TO_DEG) + "," + String(-addtheta*RAD_TO_DEG) + "," + String(R*45);
    Serial1.println(S);
    
    lastSendTime = currentTime; 
  }
}
//============================ END OF LOOP ===================================//
//========================= MPU 6050 I2C =====================================//
// left motor encoder interrupt
void left_isr() {
  if (digitalRead(leftencoder_b)) {
    leftencoder++;
    leftencoder_1++;
    leftencoder_2++;
  } else {
    leftencoder--;
    leftencoder_1--;
    leftencoder_2--;
  }
}

void righ_isr() {
  if (digitalRead(righencoder_b)) {
    righencoder--;
    righencoder_1--;
     righencoder_2--;
  } else {
    righencoder++;
    righencoder_1++;
    righencoder_2++;
  }
}
// Read psi //
void READ_MPU() {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  AcY = -((i2cData[2] << 8) | i2cData[3]);
  AcZ = -((i2cData[4] << 8) | i2cData[5]);
  Gxro = ((i2cData[8] << 8) | i2cData[9]);
  mpudata = (atan2(AcY, AcZ)) * RAD_TO_DEG;
  Gxrate = (double)Gxro / 131.0; //Tính toán tỷ lệ thay đổi góc (angular velocity) từ dữ liệu con quay hồi chuyển
  Gxangle = Gxrate * ((double)(micros() - timer) / 1000000);                      // Calculate gyro angle without any filter
  Acx = kalmanX.getAngle(mpudata, Gxrate, (double)(micros() - timer) / 1000000);  // Calculate the angle using a Kalman filter 
  float dt = (float)(micros() - timer) / 1000000.0; //trả về thời gian hiện tại tính theo micro giây. Sự chênh lệch giữa hai lần đọc thời gian được chia cho 1 triệu để chuyển sang giây.
  timer = micros();
}
//================================================================================//
//================================ H_Control =====================================//
float gettheta(long lencoder, long rencoder) {  //deg value
  float angle = 0.55 * (lencoder + rencoder);
  return angle;
}
//Read phi angle function//
float getphi(long lencoder, long rencoder) {  //deg value
  float angle = 0.268 * (lencoder - rencoder);
  return angle;
}
//LQR function//
void getlqr(float theta_, float thetadot_, float psi_, float psidot_, float phi_, float phidot_) {
  leftvolt = k1 * theta_ + k2 * thetadot_ + k3 * psi_ + k4 * psidot_ - k5 * phi_ - k6 * phidot_;// Vl,vr
  righvolt = k1 * theta_ + k2 * thetadot_ + k3 * psi_ + k4 * psidot_ + k5 * phi_ + k6 * phidot_;
  leftvolt = constrain(leftvolt, -12, 12);
  righvolt = constrain(righvolt, -12, 12);

  PWML = map(leftvolt, -12, 12, -GHPWM, GHPWM);//Limit 15 deg.
  PWMR = map(righvolt, -12, 12, -GHPWM, GHPWM);

  PWML = constrain(PWML, -250, 250);  //limit pwm value in (-240, 240) because we using high frequency pwm (31 khz)
  PWMR = constrain(PWMR, -250, 250);
}
//Motor control function//
void motorcontrol(long lpwm, long rpwm, float angle, bool stopstate) {
  if (stopstate == true) {
    stopandreset();
  } else {
    if (abs(angle) > 40) 
    {
      stopandreset();
    } else {
      
      if (leftvolt > 0.3) {
        leftmotor(abs(lpwm), 1);  //Forward
      } else if (leftvolt < -0.3) {
        leftmotor(abs(lpwm), 0);  //Back
      } else {
        stopandreset();
      }
      
      if (righvolt > 0.3) {
        righmotor(abs(rpwm), 1);
      } else if (righvolt < -0.3) {
        righmotor(abs(rpwm), 0);
      } else {
        stopandreset();
      }
    }
  }
}

void stopandreset()  
{
  leftencoder = 0;
  righencoder = 0;
  digitalWrite(leftpwm, LOW);
  digitalWrite(righpwm, LOW);
  PWML = 0;
  PWMR = 0;
  addtheta = 0;
  addphi = 0;
}

void leftmotor(uint8_t lpwm, int direct) {
  if (direct == 1) {  // angle > 0
    digitalWrite(leftmotor1, HIGH);
    digitalWrite(leftmotor2, LOW);
    analogWrite(leftpwm, lpwm);
  } else {
    digitalWrite(leftmotor1, LOW);
    digitalWrite(leftmotor2, HIGH);
    analogWrite(leftpwm, lpwm);
  }
}

void righmotor(uint8_t rpwm, int direct) {
  if (direct == 1) {  // angle > 0
    digitalWrite(righmotor1, HIGH);
    digitalWrite(righmotor2, LOW);
    analogWrite(righpwm, rpwm);
  } else {
    digitalWrite(righmotor1, LOW);
    digitalWrite(righmotor2, HIGH);
    analogWrite(righpwm, rpwm);
  }
}


void Bluetooth(){
   if (Serial1.available()) {
    inByte = Serial1.read();
    switch(inByte){
      case 'F': 
        Serial.println("F");//tien
        ForwardBack = -1;
        break;
      case 'T':
        Serial.println("T");
        ForwardBack = 0;
        break;

      case 'B':
        Serial.println("B");//lui
        ForwardBack = 1;
        break;
      case 'O':
        Serial.println("V");
        ForwardBack = 0;
        break;

      case 'R':
        Serial.println("R");//
        leftencoder_1 =85;
        righencoder_1 = -85;
        R--;
        break;
      case 'V':
        Serial.println("O");
        ForwardBack = 0;
        break;

      case 'L':
        Serial.println("L");//
        leftencoder_1 = -85;
        righencoder_1 = 85;
        R++;
        break;
      case 'U':
        Serial.println("U");
        ForwardBack = 0;
        break;
    } 
  }
}

//============================= End of file ======================================//
