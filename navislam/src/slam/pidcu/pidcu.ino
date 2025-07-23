#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Arduino.h>

MPU6050 mpu;

#define EARTH_GRAVITY_MS2 9.80665  // m/s²
#define DEG_TO_RAD        0.017453292519943295769236907684886
#define RAD_TO_DEG        57.295779513082320876798154814105

const byte pwm[] = {10, 2, 3, 13};
const byte inb[] = {14, 4, 6, 8};
const byte ina[] = {15, 5, 7, 9};
const byte encdA[] = {18, 19, 20, 21};


#define r 0.75
#define pi 3.14159265359
//float vx = 0, vy = 0, w = 0;
float L = 0.13;

float kp_ex = 0.5, ki_ex = 0.003, kd_ex = -0.001;
float kp_ey = 0.5, ki_ey = 0.003, kd_ey = -0.001;
float kp_eth = 0.1, ki_eth = 0.0005, kd_eth = -0.001;
float error_ex, error_prevex = 0, integ_ex, integ_prevex = 0;

float error_ey, error_prevey = 0, integ_ey, integ_prevey = 0;
float error_eth, error_preveth = 0, integ_eth, integ_preveth = 0;
float ex1 = 0, ey1 = 0,eth1=0;

float kp1=0.4, ki1=0.2, kd1=0.2;
float kp2=0.4, ki2=0.2, kd2=0.2;
float kp3=0.8, ki3=0.8, kd3=0;
float kp4=0.8, ki4=0.5, kd4=0;
float theta_now1 = 0, theta_prev1 = 0;
float theta_now2 = 0, theta_prev2 = 0;
float theta_now3 = 0, theta_prev3 = 0;
float theta_now4 = 0, theta_prev4 = 0;

float xd_dot, yd_dot, xd = 1, yd = -1,thetad=0;


//float x = 0, y = 0;

float vx = 0, vy = 0, w = 0;
float ui1 = 0, ui2 = 0, ui3 = 0, ui4 = 0;
float t = 0;
float dt = 0.1;
float dt1 = 0.095;

int PWMval1 = 0, PWMval2 = 0, PWMval3 = 0, PWMval4 = 0;
float motorSpeed1 = 0, motorSpeed2 = 0, motorSpeed3 = 0, motorSpeed4 = 0;
float RPM_input1 = 0, RPM_input2 = 0, RPM_input3 = 0, RPM_input4 = 0;

float error_now1, error_prev1 = 0, integ_now1, integ_prev1 = 0;
float error_now2, error_prev2 = 0, integ_now2, integ_prev2 = 0;
float error_now3, error_prev3 = 0, integ_now3, integ_prev3 = 0;
float error_now4, error_prev4 = 0, integ_now4, integ_prev4 = 0;
float Vmin = 0.01, Vmax = 5;
float PID_DC1, PID_DC2, PID_DC3, PID_DC4;
long encoderCount_prev1 = 0;
long encoderCount_prev2 = 0;
long encoderCount_prev3 = 0;
long encoderCount_prev4 = 0;

// unsigned long count1a, count2a, count3a, count4a;
double v1,v2,v3,v4;
// float dt;
float ex=0, ey=0, eth=0, yaw=0;
volatile unsigned long count1a, count2a, count3a, count4a;
// double  Vmax = 24, Vmin = 0.01;
uint8_t FIFOBuffer[64];
Quaternion q;
VectorInt16 aaWorld, ggWorld;
VectorFloat gravity;
bool DMPReady = false;


String inputString = "";     // Chuỗi lưu dữ liệu nhận được
bool stringComplete = false; // Cờ đánh dấu chuỗi đã hoàn chỉnh

void processCommand(String command); // Khai báo hàm


void setup() {
// Thiết lập MPU6050
  // Wire.begin();
  // Wire.setClock(400000);
  Serial.begin(115200);

  // mpu.initialize();
  // if (!mpu.testConnection()) {
  //   while (true);  // Dừng nếu kết nối MPU6050 thất bại
  // }

  // if (mpu.dmpInitialize() == 0) {
  //   mpu.CalibrateAccel(6);
  //   mpu.CalibrateGyro(6);
  //   mpu.setDMPEnabled(true);
  //   DMPReady = true;
  // }

  // Thiết lập các chân điều khiển động cơ
  for (int i = 0; i <= 3; i++) {
    pinMode(pwm[i], OUTPUT);
    pinMode(ina[i], OUTPUT);
    pinMode(inb[i], OUTPUT);
  }

  // Thiết lập ngắt encoder
  attachInterrupt(digitalPinToInterrupt(encdA[0]), encd1, RISING);
  attachInterrupt(digitalPinToInterrupt(encdA[1]), encd2, RISING);
  attachInterrupt(digitalPinToInterrupt(encdA[2]), encd3, RISING);
  attachInterrupt(digitalPinToInterrupt(encdA[3]), encd4, RISING);

  // delay(3000);
  stopMotors();
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    parseData(data);
    if (data == "STOP") {
      stopMotors();  // Dừng động cơ
      // delay(60000);
    } 
    else {
     Serial.print("Đã nhận: ex = "); Serial.print(ex);
     Serial.print(", ey = "); Serial.print(ey);
     Serial.print(", eth = "); Serial.println(eth);
 
  // ex=0.1;
  // ey=0.0;
  // eth=0.00;


// if (ex>=0.05 && ex<= -0.05) {ex=0.05;}
// if (ey>=0.05 && ey<= -0.05) {ey=0.05;}
// if (eth>0.1) {eth=0.1;}
// if (eth<-0.1) {eth=-0.1;}
// if(ex<=0.01 && ey<=0.01){w=0.1*eth}
// else{  w=0.6*eth;}
  vx=2*ex;
  vy=2*ey;
  w=1*eth;

  donghoc();
  PID1();
  PID2();
  PID3();
  PID4();

    }

  }
  // else{
  //   stopMotors();
  // }


}

void processCommand(String command) {
  if (command == "STOP") {
    stopMotors();
  } else {
    Serial.println("Processing command: " + command);
  }
}



void parseData(String data) {
  data.trim();
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);

  if (firstComma > 0 && secondComma > firstComma) {
    ex = data.substring(0, firstComma).toFloat();
    ey = data.substring(firstComma + 1, secondComma).toFloat();
    eth = data.substring(secondComma + 1).toFloat();
  }
}

void donghoc() {
  ui1 = (vx - vy - (w * L)) / r;
  ui2 = (vx + vy - (w * L)) / r;
  ui3 = (vx - vy + (w * L)) / r;
  ui4 = (vx + vy + (w * L)) / r;
}

void PID1() {
  RPM_input1 = abs(((ui1*r)/(pi*2*r))*60);
  //RPM_input1 = (((ui1*r)/(pi*2*r))*60);
  theta_now1 = count1a - encoderCount_prev1;
  encoderCount_prev1 = count1a;
  motorSpeed1 = (theta_now1) * 14 / (12 * 19.2 * (100 / 1000.0));
  if (ui1 > 0) {
    error_now1 = RPM_input1 - motorSpeed1;
  } else if (ui1 < 0) {
    error_now1 = RPM_input1 + motorSpeed1;
  }
  integ_now1 = integ_prev1 + (0.1 * (error_now1 + error_prev1) / 2);
  PID_DC1 = kp1*error_now1 + ki1*integ_now1 + (kd1*(error_now1 - error_prev1)/0.1);
  if (PID_DC1 > Vmax) {
    PID_DC1 = Vmax;
    integ_now1 = integ_prev1;
  }
  if (PID_DC1 < Vmin) {
    PID_DC1 = Vmin;
    integ_now1 = integ_prev1;
  }
  PWMval1 = int(100 * abs(PID_DC1) / Vmax);
  PWMval1=constrain(PWMval1,0,100);
  if (ui1 > 0) {
    digitalWrite(ina[0], HIGH);
    digitalWrite(inb[0], LOW);
    analogWrite(pwm[0], PWMval1);    
  }
  if (ui1 < 0) {
    digitalWrite(ina[0], LOW);
    digitalWrite(inb[0], HIGH);
    analogWrite(pwm[0], PWMval1);
  }
  theta_prev1 = theta_now1;
  integ_prev1 = integ_now1;
  error_prev1 = error_now1;
}
void PID2() {
  RPM_input2 = abs(((ui2*r)/(pi*2*r))*60);
  //RPM_input2 = (((ui2*r)/(pi*2*r))*60);
  theta_now2 = count2a - encoderCount_prev2;
  encoderCount_prev2 = count2a;
  motorSpeed2 = (theta_now2) * 14 / (12 * 19.2 * (100 / 1000.0));
  if (ui2 > 0) {
    error_now2 = RPM_input2 - motorSpeed2;
  } else if (ui2 < 0) {
    error_now2 = RPM_input2 + motorSpeed2;
  }

  integ_now2 = integ_prev2 + (0.1 * (error_now2 + error_prev2) / 2);
  PID_DC2 = kp1*error_now2 + ki1*integ_now2 + (kd1*(error_now2 - error_prev2)/0.1);
  if (PID_DC2 > Vmax) {
    PID_DC2 = Vmax;
    integ_now2 = integ_prev2;
  }
  if (PID_DC2 < Vmin) {
    PID_DC2 = Vmin;
    integ_now2 = integ_prev2;
  }

  PWMval2 = int(100 * abs(PID_DC2) / Vmax);
  PWMval2=constrain(PWMval2,0,100);
if (ui2 > 0) {
    digitalWrite(ina[1], HIGH);
    digitalWrite(inb[1], LOW);
    analogWrite(pwm[1], PWMval2);    
  }
  if (ui2 < 0) {
    digitalWrite(ina[1], LOW);
    digitalWrite(inb[1], HIGH);
    analogWrite(pwm[1], PWMval2);
  }
  theta_prev2 = theta_now2;
  integ_prev2 = integ_now2;
  error_prev2 = error_now2;
}
void PID3() {
  RPM_input3 = abs(((ui3*r)/(pi*2*r))*60);
  //RPM_input3 = (((ui3*r)/(pi*2*r))*60);
  theta_now3 = count3a - encoderCount_prev3;
  encoderCount_prev3 = count3a;
  motorSpeed3 = (theta_now3) * 14 / (12 * 19.2 * (100 / 1000.0));
  if (ui3 > 0) {
    error_now3 = RPM_input3 - motorSpeed3;
  } else if (ui3 < 0) {
    error_now3 = RPM_input3 + motorSpeed3;
  }
  integ_now3 = integ_prev3 + (0.1 * (error_now3 + error_prev3) / 2);
  PID_DC3 = kp1*error_now3 + ki1*integ_now3 + (kd1*(error_now3 - error_prev3)/0.1);
  if (PID_DC3 > Vmax) {
    PID_DC3 = Vmax;
    integ_now3 = integ_prev3;
  }
  if (PID_DC3 < Vmin) {
    PID_DC3 = Vmin;
    integ_now3 = integ_prev3;
  }

  PWMval3 = int(100 * abs(PID_DC3) / Vmax);
  PWMval3=constrain(PWMval3,0,100);
  if (ui3 > 0) {
    digitalWrite(ina[2], HIGH);
    digitalWrite(inb[2], LOW);
    analogWrite(pwm[2], PWMval3);    
  }
  if (ui3 < 0) {
    digitalWrite(ina[2], LOW);
    digitalWrite(inb[2], HIGH);
    analogWrite(pwm[2], PWMval3);
  }
  theta_prev3 = theta_now3;
  integ_prev3 = integ_now3;
  error_prev3 = error_now3;
}
void PID4() {
  RPM_input4 = abs(((ui4*r)/(pi*2*r))*60);
  //RPM_input4 = (((ui4*r)/(pi*2*r))*60);
  theta_now4 = count4a - encoderCount_prev4;
  encoderCount_prev4 = count4a;
  motorSpeed4 = (theta_now4) * 14 / (12 * 19.2 * (100 / 1000.0));
  if (ui4 > 0) {
    error_now4 = RPM_input4 - motorSpeed4;
  } else if (ui4 < 0) {
    error_now4 = RPM_input4 + motorSpeed4;
  }
  integ_now4 = integ_prev4 + (0.1 * (error_now4 + error_prev4) / 2);
  PID_DC4 = kp1*error_now4 + ki1*integ_now4 + (kd1*(error_now4 - error_prev4)/0.1);
  if (PID_DC4 > Vmax) {
    PID_DC4 = Vmax;
    integ_now4 = integ_prev4;
  }
  if (PID_DC4 < Vmin) {
    PID_DC4 = Vmin;
    integ_now4 = integ_prev4;
  }

  PWMval4 = int(150 * abs(PID_DC4) / Vmax);
  PWMval4=constrain(PWMval4,0,150);
  if (ui4 > 0) {
    digitalWrite(ina[3], HIGH);
    digitalWrite(inb[3], LOW);
    analogWrite(pwm[3], PWMval4);    
  }
  if (ui4 < 0) {
    digitalWrite(ina[3], LOW);
    digitalWrite(inb[3], HIGH);
    analogWrite(pwm[3], PWMval4);
  }
  theta_prev4 = theta_now4;
  integ_prev4 = integ_now4;
  error_prev4 = error_now4;
}

void handleEncoder(volatile unsigned long* count) {
  (*count)++;
}
void encd1() { handleEncoder(&count1a); }
void encd2() { handleEncoder(&count2a); }
void encd3() { handleEncoder(&count3a); }
void encd4() { handleEncoder(&count4a); }

void stopMotors() {
  analogWrite(pwm[0], 0);
  analogWrite(pwm[1], 0);
  analogWrite(pwm[2], 0);
  analogWrite(pwm[3], 0);
  Serial.println("Motors stopped!");
}
