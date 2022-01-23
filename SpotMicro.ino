#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <CommandParser.h>
#include <math.h> 

#define LED_BUILTIN 2

#define PWM_SERVO_ADDR    0x40
Adafruit_PWMServoDriver PWM = Adafruit_PWMServoDriver(PWM_SERVO_ADDR);


int pulse = 0;
int servo_id = 0;
const float DEG2RAD = PI / 180.0f;
const float RAD2DEG = 180.0f / PI;

float Lspan = 40;
float dL = 20;
float flor = -170;
float delta = 10;
float swing[10][2] = {{-Lspan, flor},
                      {-Lspan - dL, flor},
                      {-60, -140},
                      {-60, -140},
                      {-60, -140},
                      {60, -130},
                      {60, -130},
                      {60, -140},
                      {Lspan + dL, flor},
                      {Lspan, flor}};
float stance[3][2] =  {{Lspan, flor},
                      {0, flor - delta},
                      {-Lspan, flor}};      




// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // 12-Bit 16 Channel PWM Module

  PWM.begin();
  PWM.setPWMFreq(50);

  Serial.begin(115200);
  servosSleep();
  servosWake();
  robotSleep();
  delay(2000);
  for(int i = 0; i < 4;i++){
    setPosition(i, 0, -118.5);
  }
  delay(2000);
}

// the loop function runs over and over again forever
float angles[2];
unsigned long StartTime = millis();
float t = 0.0;
unsigned long CurrentTime;
float t_i;
float pos[2] = {0,0};
void loop() {
  walk(20, 200); //200mm/s
}

const float pi = 3.14159267;

float deg(float rad) {
  return rad / 2 / pi * 360;
}
float rad(float deg) {
  return deg / 360 * 2 * pi;
}

void walk(int duration, float vd)
{
  float T_stance = 2 * Lspan / vd;
  float T_swing = 0.25;
  float T_stride = T_stance + T_swing;
  float dS_trot[4] =  {0, 0.5, 0.5, 0};
  CurrentTime = millis();
  t = ((float)(CurrentTime - StartTime))/1000;
  if(fmod(t,T_stride) <= T_stride){
    for(int i = 0; i < 4; i++){
      t_i = fmod(t + T_stride * dS_trot[i],T_stride);
      Serial.printf("Current Time t_i: %f\n", t_i);
      if(t_i <= T_stance) {
        bezier(t_i / T_stance, stance, 3, pos);
      }
      else {
        bezier((t_i - T_stance) / T_swing, swing, 10, pos);
      }
      setPosition(i,pos[0],pos[1]);
    }
  }
  if(t > duration) {
    robotSleep();
//    servosSleep();
    while(1);
  }
}

void inverseKin(float x, float y, float arr[2]) {
  // alpha_1
  arr[1] = (acos((pow(x,2) + pow(y,2) - pow(111.126,2) - pow(118.5,2)) / (2 * 111.126 * 118.5)));
  arr[0] = (atan2(y, x) - atan2(118.5 * sin(arr[1]), 111.126 + 118.5 * cos(arr[1])));
  arr[1] = RAD2DEG * arr[1];
  arr[0] = RAD2DEG * arr[0];
//  Serial.printf("a1: %f, a2: %f\n", arr[0], arr[1]);
}

//void bodyIK(float omega, float phi, float psi, float xm, float ym, float zm) {
//  float Rx = 
//}
float l1 = 60.5;
float l2 = 10;
float l3 = 111.126;
float l4 = 118.5;

void legIK(float theta[3], float x, float y, float z) {
    float F= sqrt(pow(x,2)+pow(y,2)-pow(l1,2));
    float G=F-l2  ;
    float H=sqrt(pow(G,2)+pow(z,2));

    float theta1= -atan2(y,x)-atan2(F,-l1);

    float D=(pow(H,2)-pow(l3,2)-pow(l4,2))/(2*l3*l4);
    float theta3=acos(D);

    float theta2=atan2(z,G)-atan2(l4*sin(theta3),l3+l4*cos(theta3));

    theta[0] = RAD2DEG *(theta1);
    theta[1] = RAD2DEG *(theta2);
    theta[2] = RAD2DEG *(theta3);
}


void servosSleep() {
  PWM.sleep();
}

void servosWake() {
  PWM.wakeup();
}

void writePWM(int servo_id, int pulse){
//  Serial.printf("Servo %d: Pulse %d\n", servo_id, pulse);
  PWM.setPWM(servo_id, 0, pulse);
  
}

int sign(float num) {
  return (num >= 0) - (num < 0);
}

void commandFL(float angles[2]) {
  float a1 = (sign(angles[0])*180 - angles[0])+15;
  int pulseLength1 = map(a1, -90,90,180,470);
  float a2 = -(180-angles[1]) +15;
  int pulseLength2 = map(a2,-180,0,390,105);
  writePWM(0,325);
  writePWM(1,pulseLength1);
  writePWM(2,pulseLength2);
}

void commandFR(float angles[2]) {
  float a1 = (sign(angles[0])*180 - angles[0])+15; //should be a +15 here, but something is up
  int pulseLength1 = map(a1, -90,90,470,185);
  float a2 = -(180-angles[1]) +15;
  int pulseLength2 = map(a2,-180,0,265,555);
  writePWM(3,270);
  writePWM(4,pulseLength1);
  writePWM(5,pulseLength2);
}

void commandBR(float angles[2]) {
//  float a1 = angles[0];
//  int pulseLength1 = map(a1,-30,90,360,165);
  float a2 = (sign(angles[0])*180 - angles[0])+15;
  int pulseLength2 = map(a2, -90,90,445,165);
  float a3 = -(180-angles[1])+5;
  int pulseLength3 = map(a3,-180,0,110,390);
  writePWM(6,307);
  writePWM(7,pulseLength2);
  writePWM(8,pulseLength3);
}

void commandBL(float angles[2]) {
  float a1 = (sign(angles[0])*180 - angles[0])+15;
  int pulseLength1 = map(a1, -90,90,167,452);
  float a2 = -(180-angles[1]) +15;
  int pulseLength2 = map(a2,-180,0,400,110);
  writePWM(9,335);
  writePWM(10,pulseLength1);
  writePWM(11,pulseLength2);
}

void setPosition(int legnum, int x, int y) {
  float angles[2];
  inverseKin(x,y,angles);
  switch(legnum){
    case 0:
      commandFL(angles);
      break;
    case 1:
      commandFR(angles);
      break;
    case 2:
      commandBL(angles);
      break;
    case 3:
      commandBR(angles);
      break;
    default:
      break;
  }
}

void robotSleep() {
  writePWM(0,325);
  writePWM(1,320);
  writePWM(2,105);

  writePWM(3,270);
  writePWM(4,325);
  writePWM(5,545);

  writePWM(6,307);
  writePWM(7,307);
  writePWM(8,390);

  writePWM(9,335);
  writePWM(10,307);
  writePWM(11,110);
}

int factorial(int n){
  if (n == 1 || n == 0){
    return 1;
  }
  else{
    return n * factorial(n - 1);
  }
}

int choose(int n, int k) {
  return factorial(n) / (factorial(k) * factorial(n - k));
}


float bezArray[12][12] = {
  {1,0,0,0,0,0,0,0,0,0,0,0},
  {1,1,0,0,0,0,0,0,0,0,0,0},
  {1,2,1,0,0,0,0,0,0,0,0,0},
  {1,3,3,1,0,0,0,0,0,0,0,0},
  {1,4,6,4,1,0,0,0,0,0,0,0},
  {1,5,10,10,5,1,0,0,0,0,0,0},
  {1,6,15,20,15,6,1,0,0,0,0,0},
  {1,7,21,35,35,21,7,1,0,0,0,0},
  {1,8,28,56,70,56,28,8,1,0,0,0},
  {1, 9, 36, 84, 126, 126, 84, 36, 9, 1,0,0},
  {1, 10, 45, 120, 210, 252, 210, 120, 45, 10, 1,0},
  {1, 11, 55, 165, 330, 462, 462, 330, 165, 55, 11, 1}
};

void bezier(float t, float control_points[][2], size_t len, float traj[2]) {
  float x = 0;
  float y = 0;
  float bez = 0;
  for(size_t i = 0; i < len; i++)  {
//    bez = choose(len - 1, (int)i) * pow((1 - t),(len - 1 - (int)i)) * pow(t,(int)i);
    bez = bezArray[len-1][i] * pow((1 - t),(len - 1 - (int)i)) * pow(t,(int)i);
    x += bez * control_points[i][0];
    y += bez * control_points[i][1];
  }
//  Serial.printf("Setting Position (%f,%f)\n",x,y);
  traj[0] = x;
  traj[1] = y;
}
