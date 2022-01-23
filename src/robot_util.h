#include <math.h>
#include <Adafruit_PWMServoDriver.h>

#define PWM_SERVO_ADDR    0x40
Adafruit_PWMServoDriver PWM = Adafruit_PWMServoDriver(PWM_SERVO_ADDR);

const float DEG2RAD = PI / 180.0f;
const float RAD2DEG = 180.0f / PI;

int pulse = 0;
int servo_id = 0;


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

void startPWM() {
  PWM.begin();
  PWM.setPWMFreq(50);
}

void writePWM(int servo_id, int pulse) {
//  Serial.printf("Servo %d: Pulse %d\n", servo_id, pulse);
  PWM.setPWM(servo_id, 0, pulse);

}

void servosSleep() {
  PWM.sleep();
}

void servosWake() {
  PWM.wakeup();
}

int sign(float num) {
  return (num >= 0) - (num < 0);
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

void inverseKin(float x, float y, float arr[2]) {
  // alpha_1
  arr[1] = (acos((pow(x,2) + pow(y,2) - pow(111.126,2) - pow(118.5,2)) / (2 * 111.126 * 118.5)));
  arr[0] = (atan2(y, x) - atan2(118.5 * sin(arr[1]), 111.126 + 118.5 * cos(arr[1])));
  arr[1] = RAD2DEG * arr[1];
  arr[0] = RAD2DEG * arr[0];
//  Serial.printf("a1: %f, a2: %f\n", arr[0], arr[1]);
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
