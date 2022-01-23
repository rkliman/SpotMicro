#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <spot_math.h>
#include <robot_util.h>

#define LED_BUILTIN 2

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

// the loop function runs over and over again forever
float angles[2];
unsigned long StartTime = millis();
float t = 0.0;
unsigned long CurrentTime;
float t_i;
float pos[2] = {0,0};
void walk(int duration, float vd) {
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

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // 12-Bit 16 Channel PWM Module

  startPWM();

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


void loop() {
  walk(20, 200); //200mm/s
}
