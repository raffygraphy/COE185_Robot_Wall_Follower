#include "motor.h"

void initMotorDirection() {

  // Set pins as output
  DDRD |= (1 << LEFT_PIN);   //D2
  DDRD |= (1 << RIGHT_PIN);  //D4
}

void initPWMsignal() {

  // Set pins as output
  DDRD |= (1 << LEFT_PWM_PIN);   //D5
  DDRD |= (1 << RIGHT_PWM_PIN);  //D6
}

void init_Timer() {

  // MODE TIMER 8-BIT
  TCCR0A |= (1 << WGM00);
  TCCR0A |= (1 << WGM01);

  // Set prescaler to 1 (no prescaling)
  TCCR0B |= (1 << CS00);


  // Clear OC0A and OC0B on compare match, set at BOTTOM
  TCCR0A |= (1 << COM0A1) | (1 << COM0B1);
}

void configure_Motor(side_Wheel side_W, double powerFactor, direction_Value value) {

  // left
  if (side_W == left_W) {
    OCR0A = MAX_PWM / powerFactor;
    if (value == high_value) {
      PORTD |= (1 << LEFT_PIN);
    } else {
      PORTD &= ~(1 << LEFT_PIN);
    }
    //right
  } else if (side_W == right_W) {
    OCR0B = MAX_PWM / powerFactor;
    if (value == high_value) {
      PORTD |= (1 << RIGHT_PIN);
    } else {  //low
      PORTD &= ~(1 << RIGHT_PIN);
    }
  }
}

void stop() {
  OCR0A = 0;
  OCR0B = 0;
  delay(500);
}
