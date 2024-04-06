#ifndef MOTOR_H
#define motor_H

#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>

#define LEFT_PWM_PIN DDD5   //DIG5
#define RIGHT_PWM_PIN DDD6  //DIG6

#define LEFT_PIN DDD2   //DIG2
#define RIGHT_PIN DDD4  //DIG4

#define MAX_PWM 255   // Maximum PWM value

typedef enum {
  left_W,
  right_W
} side_Wheel;

typedef enum {
  high_value,
  low_value
} direction_Value;

void initMotorDirection();
void initPWMsignal();
void init_Timer();

void configure_Motor(side_Wheel side_W, double powerFactor, direction_Value value);
void stop();

#endif