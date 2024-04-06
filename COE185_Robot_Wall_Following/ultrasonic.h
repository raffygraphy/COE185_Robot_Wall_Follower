#ifndef ULTRASONIC_H
#define ultrasonic_H

#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>

#define LED_LEFT PB2   // Pin 10 (PB0) for left LED
#define LED_RIGHT PB3  // Pin 11 (PB1) for right LED

#define trigPin PB4  // Pin 8 (PB0) for
#define echoPin PB5  // Pin 9 (PB1) for

void initPWM();
void initUltraSonic();
void initLED();
void triggerUltrasonicSensor();
void moveTheServo(int duty_cycle);
double readEcho();
void toggleLED(String position);
void offLED(String position);

void center_To_Left();
void left_To_Center();
void center_To_Right();
void right_To_Center();

double read_only_ULTRASONIC();
#endif