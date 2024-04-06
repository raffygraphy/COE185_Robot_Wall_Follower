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


// typedef enum {
//   CENTER,
//   RIGHT_side
// } idealDirection;


int maintaining_Distance = 6;
int threshold = 2;

const double originalSpeed = 0.3;
const double distanceThreshold = 12;
double percentSpeed = originalSpeed;
unsigned long t = 0;

#define right_View 100
#define center_View 380
#define left_View 625

int primary_View = center_View;

bool firstWall = false;
bool secondWall = false;

double firstRead = 0;

double measured_value_center = 0;

void setup() {
  Serial.begin(9600);
  initPWM();
  initUltraSonic();
  initLED();

  initMotorDirection();
  initPWMsignal();
  init_Timer();
  // OCR1A = 100;


  swivelMeasurement(left_View);
  swivelMeasurement(center_View);
  swivelMeasurement(right_View);
  // Serial.println(firstRead);
  delay(1000);
  t = millis();
}

void wall_follow(double powerFactor, double maintainDistance, side_Wheel side_W) {
  Serial.println("Wall Follow");
  double distance = read_only_ULTRASONIC();
  if (distance > 30) {
    distance = maintainDistance;
  }
  if (side_W == right_W) {

    configure_Motor(right_W, (1 / (powerFactor * (distance / maintainDistance))), low_value);  //d4 low   percentPower * (distance / maintainDistance
    configure_Motor(left_W, (1 / (powerFactor * (maintainDistance / distance))), high_value);  //d2 high percentPower * (maintainDistance / distance)

  } else if (side_W == left_W) {
    configure_Motor(left_W, (1 / (powerFactor * (distance / maintainDistance))), high_value);  //d2 high
    configure_Motor(right_W, (1 / (powerFactor * (maintainDistance / distance))), low_value);  //d4 low
  }
  delay(60);
  Serial.println(distance);
}


void right() {
  if (primary_View == right_View) {
    configure_Motor(left_W, (1 / percentSpeed), high_value);  //d2 high percentPower * (maintainDistance / distance)
    configure_Motor(right_W, (1 / percentSpeed), low_value);  //d4 low   percentPower * (distance / maintainDistance

    double measured_value = swivelMeasurement(right_View);
    Serial.print("Right Value is: ");
    Serial.println(measured_value);

    if (measured_value > 3 * distanceThreshold && measured_value < 30) {
      Serial.print("Towards wall");

      configure_Motor(left_W, (MAX_PWM + 1), high_value);  //d2 high percentPower * (maintainDistance / distance)
      configure_Motor(right_W, (MAX_PWM + 1), low_value);  //d4 low   percentPower * (distance / maintainDistance
      delay(50);
      configure_Motor(left_W, (1 / 0.34), high_value);  //d2 high percentPower * (maintainDistance / distance)
      configure_Motor(right_W, (1 / 0.4), low_value);   //d4 low   percentPower * (distance / maintainDistance
      delay(1000);

      configure_Motor(left_W, (MAX_PWM + 1), high_value);  //d2 high percentPower * (maintainDistance / distance)
      configure_Motor(right_W, (MAX_PWM + 1), low_value);  //d4 low   percentPower * (distance / maintainDistance
      delay(50);

      //turn right

      configure_Motor(left_W, (1 / percentSpeed), high_value);   //d2 high percentPower * (maintainDistance / distance)
      configure_Motor(right_W, (1 / percentSpeed), high_value);  //d4 low   percentPower * (distance / maintainDistance
      delay(400);
      configure_Motor(left_W, (MAX_PWM + 1), low_value);   //d2 high percentPower * (maintainDistance / distance)
      configure_Motor(right_W, (MAX_PWM + 1), low_value);  //d4 low   percentPower * (distance / maintainDistance
      delay(50);

      primary_View = center_View;
      // swivelMeasurement(center_View);
      firstWall = true;
    }
  } else {
    if ((!firstWall && !secondWall)) {
      Serial.println("No Walls yet");

      //for 3 seconds follow the wall
      if (millis() - t < 2000) {
        Serial.println("Walk Muna for 3 seconds");
        wall_follow(percentSpeed, distanceThreshold, right_W);
      } else {
        // pause walking
        configure_Motor(left_W, (MAX_PWM + 1), low_value);    //d2 high percentPower * (maintainDistance / distance)
        configure_Motor(right_W, (MAX_PWM + 1), high_value);  //d4 low   percentPower * (distance / maintainDistance

        measured_value_center = swivelMeasurement(center_View);
        Serial.println(measured_value_center);

        // delay(50);

        if (measured_value_center < distanceThreshold) {
          if (!firstWall) {
            Serial.println("Wall in Center");
            delay(3000);
            configure_Motor(left_W, (1 / percentSpeed), low_value);   //d2 low
            configure_Motor(right_W, (1 / percentSpeed), low_value);  //d4 low
            delay(500);
            configure_Motor(left_W, MAX_PWM + 1, high_value);   //d2 high
                                                                // configure_Motor(right_W, MAX_PWM + 1, low_value);  //d4 low
            configure_Motor(right_W, MAX_PWM + 1, high_value);  //d4 low
            delay(500);

            Serial.println("Turning Left");
            // swivelMeasurement(right_View);
            primary_View = right_View;
            firstWall = true;
          }
        }
        Serial.println("No Wall @ Center");
        // shift_center_To_Right();
        moveTheServo(right_View);
        // t = millis();
        
      }
    } else if (firstWall && secondWall) {  // If we're past all the obstacles, we don't need to check ahead) {
                                           // } else if (firstWall) {  // If we're past all the obstacles, we don't need to check ahead) {
      wall_follow(percentSpeed, distanceThreshold, right_W);
    } else {

      //forward
      configure_Motor(left_W, (1 / percentSpeed), high_value);  //d2 high percentPower * (maintainDistance / distance)
      configure_Motor(right_W, (1 / percentSpeed), low_value);  //d4 low   percentPower * (distance / maintainDistance

      measured_value_center = swivelMeasurement(center_View);
      if (measured_value_center < distanceThreshold) {

        configure_Motor(left_W, MAX_PWM + 1, low_value);    //d2 high
        configure_Motor(right_W, MAX_PWM + 1, high_value);  //d4 low
        delay(500);
        configure_Motor(left_W, (1 / percentSpeed), high_value);   //d2 high
        configure_Motor(right_W, (1 / percentSpeed), high_value);  //d4 high
        delay(300);
        configure_Motor(left_W, MAX_PWM + 1, low_value);   //d2 low
        configure_Motor(right_W, MAX_PWM + 1, low_value);  //d4 low

        delay(50);
        primary_View = right_View;
      }
    }
    Serial.println("Time to Check Right");
  }
}


double centermeasure;
double rightmeasure;
void loop() {
  right();
}

double measured;
double swivelMeasurement(int directions) {

  moveTheServo(directions);
  measured = read_only_ULTRASONIC();
  delay(60);
  return measured;
}

void shift_right_To_Center() {
  // right to center
  for (int i = 0; i <= 375; i += 2) {  // 0 degrees: 1.5 ms pulse, 90 degrees: 2.5 ms pulse
    Serial.println(i);
    moveTheServo(i);  // Move servo to current angle
  }

  _delay_ms(500);  // Wait for 5 ms
}

void shift_center_To_Right() {
  // center to right
  for (int i = 375; i >= 0; i -= 2) {  // 0 degrees: 1.5 ms pulse, 90 degrees: 2.5 ms pulse
    Serial.println(i);
    moveTheServo(i);  // Move servo to current angle
  }
  _delay_ms(500);  // Wait for 5 ms
}

void shift_left_to_center() {
  // center to right
  for (int i = left_View; i >= center_View; i -= 50) {  // 0 degrees: 1.5 ms pulse, 90 degrees: 2.5 ms pulse
    Serial.println(i);
    moveTheServo(i);  // Move servo to current angle
  }
  _delay_ms(500);  // Wait for 5 ms
}
