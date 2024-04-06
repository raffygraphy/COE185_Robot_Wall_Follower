#include <Arduino.h>
#include "ultrasonic.h"
#include "motor.h"
#include <avr/io.h>
#include <util/delay.h>

#define LEFT_PWM_PIN DDD5   //DIG5
#define RIGHT_PWM_PIN DDD6  //DIG6

#define LEFT_PIN DDD2   //DIG2
#define RIGHT_PIN DDD4  //DIG4

#define LED_LEFT PB2   // Pin 10 (PB0) for left LED
#define LED_RIGHT PB3  // Pin 11 (PB1) for right LED

#define trigPin PB4  // Pin 8 (PB0) for
#define echoPin PB5  // Pin 9 (PB1) for


int maintaining_Distance = 6;
int threshold = 2;

void setup() {
  Serial.begin(9600);
  initPWM();
  initUltraSonic();
  initLED();

  initMotorDirection();
  initPWMsignal();
  init_Timer();

  delay(2000);
}

void wall_follow(double powerFactor, double maintainDistance, side_Wheel side_W) {
  double distance = read_only_ULTRASONIC();
  if (distance > 30) {
    distance = maintainDistance;
  }
  if (side_W == right_W) {
    configure_Motor(left_W, (distance / (maintainDistance * powerFactor)), high_value);  //d2 high percentPower * (maintainDistance / distance)
    configure_Motor(right_W, (maintainDistance / (powerFactor * distance)), low_value);  //d4 low   percentPower * (distance / maintainDistance

  } else if (side_W == left_W) {

    configure_Motor(right_W, (distance / (maintainDistance * powerFactor)), low_value);  //d4 low
    configure_Motor(left_W, (maintainDistance / (powerFactor * distance)), high_value);  //d2 high
  }
  delay(60);
  Serial.println(distance);
}

void loop() {
  wall_follow(0.3, 10, right_W);
}
