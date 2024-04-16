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


const double originalSpeed = 0.2;
double percentSpeed = originalSpeed;

#define right_View 100
#define center_View 380
#define left_View 625
#define mid_view 200


void setup() {
  Serial.begin(9600);
  initPWM();
  initUltraSonic();
  initLED();

  initMotorDirection();
  initPWMsignal();
  init_Timer();
  initServos();
  delay(1000);
}

double threshold = 2.0;
double maintainDistance = 8;

void wall_follow(double powerFactor, double maintainDistance, ultrasonic_sight view) {
  double distance = read_only_ULTRASONIC();

  if (distance < (maintainDistance + threshold) && distance > (maintainDistance - threshold)) {
    Serial.println("Right Place");
    if (view == mid_sight) {
      configure_Motor(right_W, (1 / (powerFactor * (distance / maintainDistance))), low_value);  //d4 low   percentPower * (distance / maintainDistance
      configure_Motor(left_W, (1 / (powerFactor * (maintainDistance / distance))), high_value);  //d2 high percentPower * (maintainDistance / distance)

      Serial.println(distance);
    }
    if (view == right_sight) {
      configure_Motor(right_W, (1 / (powerFactor * (distance / maintainDistance))), low_value);  //d4 low   percentPower * (distance / maintainDistance
      configure_Motor(left_W, (1 / (powerFactor * (maintainDistance / distance))), high_value);  //d2 high percentPower * (maintainDistance / distance)

      Serial.println(distance);
    }
  } else if (distance > (maintainDistance + threshold)) {

    Serial.println("Move Towards The Wall");
    Serial.println(distance);
    rotate_right();
    distance = maintainDistance;
    if (view == mid_sight) {
      configure_Motor(right_W, (1 / ((powerFactor) * (distance / maintainDistance))), low_value);  //d4 low   percentPower * (distance / maintainDistance
      configure_Motor(left_W, (1 / ((powerFactor) * (maintainDistance / distance))), high_value);  //d2 high percentPower * (maintainDistance / distance)

      Serial.println(distance);
    }
    if (view == right_sight) {
      configure_Motor(right_W, (1 / (powerFactor * (distance / maintainDistance))), low_value);  //d4 low   percentPower * (distance / maintainDistance
      configure_Motor(left_W, (1 / (powerFactor * (maintainDistance / distance))), high_value);  //d2 high percentPower * (maintainDistance / distance)

      swivelMeasurement(mid_view);
      wall_follow(percentSpeed, maintainDistance, mid_sight);
    }


  } else if (distance < (maintainDistance - threshold)) {
    Serial.println("Move Away The Wall");
    Serial.println(distance);

    moveBackward();
    rotate_left();
    delay(1000);
    distance = maintainDistance;
    configure_Motor(right_W, (1 / (powerFactor * (distance / maintainDistance))), low_value);  //d4 low   percentPower * (distance / maintainDistance
    configure_Motor(left_W, (1 / (powerFactor * (maintainDistance / distance))), high_value);  //d2 high percentPower * (maintainDistance / distance)

    // rotate_left();
  }
}


unsigned long previousMillis = 0;  // will store the last time the loop was executed
const long interval = 1000;        // interval at which to exit the loop (in milliseconds)

void loop() {

  swivelMeasurement(mid_view);
  wall_follow(percentSpeed, maintainDistance, mid_sight);
}

void moveForward() {
  Serial.println("Move Forward For Space");  // d2:d4 == high;high
  configure_Motor(left_W, 3, high_value);    //d2 high
  configure_Motor(right_W, 3, low_value);    //d4 low
  delay(350);
  configure_Motor(left_W, MAX_PWM + 1, low_value);    //d2 low
  configure_Motor(right_W, MAX_PWM + 1, high_value);  //d4 high
  stop();
}

void pause() {
  configure_Motor(left_W, MAX_PWM + 1, low_value);    //d2 low
  configure_Motor(right_W, MAX_PWM + 1, high_value);  //d4 high
}

void swivelMeasurement(int directions) {
  moveTheServo(directions);
}

void moveBackward() {
  Serial.println("Move Back For Space");    // d2:d4 == high;high
  configure_Motor(left_W, 5, low_value);    //d2 high
  configure_Motor(right_W, 5, high_value);  //d4 low
  delay(350);

  configure_Motor(left_W, MAX_PWM + 1, high_value);  //d2 low
  configure_Motor(right_W, MAX_PWM + 1, low_value);  //d4 high
  pause();
}

void rotate_left() {
  configure_Motor(left_W, 2.5, low_value);   //d2 high
  configure_Motor(right_W, 2.5, low_value);  //d4 low
  delay(250);
  configure_Motor(left_W, MAX_PWM + 1, high_value);   //d2 high
  configure_Motor(right_W, MAX_PWM + 1, high_value);  //d4 high
}


void rotate_right() {
  configure_Motor(left_W, 2.5, high_value);   //d2 high
  configure_Motor(right_W, 2.5, high_value);  //d4 high
  delay(150);
  configure_Motor(left_W, MAX_PWM + 1, low_value);   //d2 low
  configure_Motor(right_W, MAX_PWM + 1, low_value);  //d4 low
}

void initServos() {
  moveTheServo(375);  //center
  delay(50);
  moveTheServo(100);  //right
  delay(50);
  moveTheServo(225);  //half  Right
  delay(50);
  moveTheServo(0);  //resident
}
