#include <Arduino.h>
#include "drive.h"
#include "CAN.h"
#include "RobomasMotor.h"
#include "Encoder.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Metro.h>
#include <SPI.h>
#include "DualShock.h"
#include "function.h"
#include "FlexCAN.h"
#include "1.Operate.h"

#define HWSERIAL Serial1
#define MOTOR_COUNT 4
#define MOTOR_CANBUS 0
#define COMMAND_CANBUS 1

#define CONTROL_CYCLE 1.0 /*ms*/
#define MOTOR_CYCLE 2.0

#define SMOOTH_SLOW 1
#define SMOOTH_NORMAL 10
#define RISING_PWM 0
#define SMALL_PWM 10

#define ENCODER_X_A 9
#define ENCODER_X_B 7
#define ENCODER_Y_A 26
#define ENCODER_Y_B 27

#define LED1 2
#define GYAIRO_LED 24
#define BUTTON 13
#define BUTTON_PUSH 0

#define IS(x) digitalRead(x) == BUTTON_PUSH

#define GYAIRO_CYCLE 1
#define ENCODER_CYCLE 1
#define DISP_CYCLE 10
#define LED_CYCLE 500

short drive_gain[MOTOR_COUNT][3] = {
  { 1,-1,-1},
  { 1, 1,-1},
  {-1, 1,-1},
  {-1,-1,-1},
};

double angle[4] = {0};
double x_rate[2],y_rate[2];
double sita1;
double diff = 0;
int count = 0;

double drive_PIDx_gain[3] = {9.0,0.0,0};
double drive_PIDy_gain[3] = {9.0,0.0,0};
double drive_PIDr_gain[3] = {1.7,0.01,0.5};

double M_Pg = 5.342;
double M_Ig = 4.21;
double M_Dg = 2.4;

double targets_rpm[4] = {0};//速さ(Hz)
double gyro_sence;
double x_val;
double y_val;
double x_keep[2];
double y_keep[2];
// double lx = 0;
// double ly = 0;
// double rx = 0;
// double lxbf = 0;
// double lybf = 0;
// double rxbf = 0;
// double lxaf = 0;
// double lyaf = 0;
// double rxaf = 0;


bool rotation_sign;
int8_t rotation_num=0;
double rotation_delta=0;
float rotation_keep[10];
CAN_message_t sendM;

int data[10] = {0};
int monitoring = 0; 

uint8_t led_count = 0;




CanControl* drive_can = CanControl::CreateCanControl(MOTOR_CANBUS);
//CanControl* command_can = CanControl::CreateCanControl(COMMAND_CANBUS);
Motor motor(MOTOR_COUNT,SMOOTH_SLOW,SMOOTH_NORMAL,RISING_PWM,SMALL_PWM);
Drive AutoDrive(motor,CONTROL_CYCLE/1000);
DJIMotor SpdControl(drive_can,CONTROL_CYCLE/1000,MOTOR_COUNT);

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

Encoder enc_x(ENCODER_X_A,ENCODER_X_B);
Encoder enc_y(ENCODER_Y_A,ENCODER_Y_B);

Metro controlTimming(CONTROL_CYCLE);
Metro motorTimming(MOTOR_CYCLE);
Metro gyaroTimming(GYAIRO_CYCLE);
Metro encoderTimming(ENCODER_CYCLE);
Metro dispTimming(DISP_CYCLE);
Metro ledTimming(LED_CYCLE);
Metro Serialtiming = Metro(1);
Metro idoutiming(4000);///////////
//変更点


void pinModeSet();
void getpalam();
void getEncorder();
void printD();

void timer(void);
void control_data_receive(int recive);

void setup() {
  delay(500);
  Serial.begin(38400);
  // Serial.begin(9600);
  HWSERIAL.begin(9600);
  SpdControl.init();
  pinModeSet();
  if(!bno.begin())while(1);
  delay(500);
  bno.setExtCrystalUse(true);
  for(int i=0;i<MOTOR_COUNT;i++)
  AutoDrive.setDriveGain(drive_gain);
  AutoDrive.setDrivePID(drive_PIDx_gain,drive_PIDy_gain,drive_PIDr_gain);
  for(int i=1;i<=MOTOR_COUNT;i++){
    SpdControl.setPIDgain(i,M_Pg,M_Ig,M_Dg);
  }
  delay(500);
}

void pinModeSet(){
  pinMode(LED1,OUTPUT);
  pinMode(GYAIRO_LED,OUTPUT);
  pinMode(BUTTON,INPUT_PULLUP);
}
int con=0;
double idou[10][3] = {0};
double damy[10][3] = {0};
int c=0;
int m=-1; 
double gyro_z = 0;//旋回角の変数
int enc[2] = {};//エンコーダの配列

void loop() {
  int incomingByte;

  if (HWSERIAL.available() > 0) {
          incomingByte = HWSERIAL.read();
          control_data_receive(incomingByte);
  }
  Auto(1);















  count = Idou(0,damy,idou);

  sita1 = gyro_sence-PI/4;
  if(sita1>PI)sita1 = sita1 - 2*PI;

  AutoDrive.now.r = gyro_sence;
	AutoDrive.searchPosition(x_val,y_val,sita1);//gyro_sence+PI/4

  AutoDrive.to = Point(idou[c][0],idou[c][1],idou[c][2]);

  
  diff = abs(AutoDrive.to.x - AutoDrive.now.x) + abs(AutoDrive.to.y - AutoDrive.now.y);
  if(circle_button == 1){
    if(diff < 3)c++;
    if(c == count)c=0;
  }
  

    AutoDrive.absoluteMove();
    AutoDrive.update();


    delay(1);//変更点
  
  for(int i=1;i<=MOTOR_COUNT;i++){
    SpdControl.setTargetRPM(i,AutoDrive.motor[i-1]);
  }

  if(NOMAL_DROVE){
    if(NOMAL_MOVE)gyro_sence = 0;
    anomalyDriveing(joystick_lx, -joystick_ly, joystick_rx, AutoDrive.motor, gyro_sence);
    for(int i=1;i<=MOTOR_COUNT;i++){
      if(joystick_lx != 0 && joystick_ly != 0 && joystick_rx != 0)AutoDrive.motor[i-1] = 0;
      SpdControl.setTargetRPM(i,AutoDrive.motor[i-1]);
    }
    
  }

  if(controlTimming.check()){
    SpdControl.speedControl(angle);
    for(int kkk=0;kkk<4;kkk++){
      if(angle[kkk] >= 4096)angle[kkk] = angle[kkk] - 8191;	
      angle[kkk] = angle[kkk] / 8190;
    }
    Angle(angle);
    for(int ppp=0;ppp<2;ppp++){
        y_rate[ppp] = angle[2*ppp+1]*2*PI*55/19;
        x_rate[ppp] = angle[2*ppp]*2*PI*55/19;
    }
    x_val = (x_rate[0]-x_rate[1])/2;
    y_val = (y_rate[0]-y_rate[1])/2;
    x_val = x_val / 30;
    y_val = y_val / 30;
  }
  getpalam();
  // getEncorder();
  if(IS(BUTTON)){
  }
  if(dispTimming.check()){
    printD();
  }
  if(ledTimming.check()){
    digitalWrite(GYAIRO_LED,led_count++%2);
    if(led_count>1000)led_count=0;
  }
}

void getpalam(void){
  //Calibration status values: 0=uncalibrated, 3=fully calibrated
  imu::Vector<3> euler;
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  imu::Quaternion quat = bno.getQuat();
  euler = quat.toEuler();
  //get Axis Data

  double r=euler[0];
  rotation_keep[0] = r;
  rotation_sign = rotation_keep[9] <= rotation_keep[0];
  if(rotation_keep[0]*rotation_keep[9]<0){
    if(PI - fabs(rotation_keep[9]) < 0.1)rotation_sign = rotation_keep[9] > 0;
    //else if(fabs((rotation_num)*PI-gyro_sence) < 0.1)rotation_num += rotation_keep[1] < 0 ? 1:-1;
  }
  if(r*gyro_sence < 0 && fabs(r)>0.1){
    if(r>0)rotation_delta = -2*PI;
    else   rotation_delta = 2*PI ;
  }else{
    rotation_delta = 0;
  }
  
  gyro_sence = r + rotation_delta + 2*PI*rotation_num ;
  for(int i=9;i>0;i--){
    rotation_keep[i] = rotation_keep[i-1];
  }
}

void getEncorder(){
    x_keep[0] = -enc_x.read()/1000.0;
    x_val = x_keep[0] - x_keep[1];
    x_keep[1] = x_keep[0];
    y_keep[0] = enc_y.read()/1000.0;
    y_val = y_keep[0] - y_keep[1];
    y_keep[1] = y_keep[0];

}

void printD(){
    Serial.print(joystick_lx);
    Serial.print("  ");
    Serial.print(joystick_ly);
    Serial.print("  ");
    Serial.print(joystick_rx);

    Serial.println();
}

void timer(void){
    monitoring++;
    if(monitoring>=200){
        circle_button   = 0;
        cross_button    = 0;
        triangle_button = 0;
        square_button   = 0;
        left_button  = 0;
        right_button = 0;
        up_button    = 0;
        down_button  = 0;
        l1_button = 0;
        l2_button = 0;
        l3_button = 0;
        r1_button = 0;
        r2_button = 0;
        r3_button = 0;
        ps_button = 0;
        start_button  = 0;
        select_button = 0;
        joystick_rx = 0;
        joystick_ry = 0;
        joystick_lx = 0;
        joystick_ly = 0;
    }
}
int No = 0;
void control_data_receive(int recive){
  if(recive == 0x80){
    No = 0;
    data[No++] = 0x80;
  }else if(No > 0){
    data[No++] = recive;
    if(No > 8){
      updataState(data);
    }
  }
}