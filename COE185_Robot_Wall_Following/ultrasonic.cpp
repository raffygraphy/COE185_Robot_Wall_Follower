#include "ultrasonic.h"

double distance = 0;

// void setup() {
//   Serial.begin(9600);
//   initPWM();
//   initUltraSonic();
//   initLED();
//   // sei();  // E
// }

void initPWM() {
  // MODE TIMER
  TCCR1B &= ~(1 << WGM13);
  TCCR1B |= (1 << WGM12);

  // PWM MODE 15BIT OF THE TIMER
  TCCR1A |= (1 << WGM13) | (1 << WGM12) | (1 << WGM11) | (1 << WGM10);
  //   // MODE TIMER
  // TCCR1B &= ~(1 << WGM13);
  // TCCR1B |= (1 << WGM12);

  // // PWM MODE 10BIT OF THE TIMER
  // TCCR1A |= (1 << WGM11) | (1 << WGM10);

  // PWM MODE 10
  TCCR1A |= (1 << COM1A1);
  TCCR1A &= ~(1 << COM1A0);  //NON INVERTING MODE

  // Set prescaler to 64
  TCCR1B |= (1 << CS11) | (1 << CS10);  // 12,11,10 - 100

  /*  
  System clock is 16Mhz sdand prescalar is "n" for generating clock of 1s
  Speed of Timer1 = 16MHz/8 = 2MHz, Pulse Time= 1/2MHz KHz = 0.5μs
  Speed of Timer1 = 16MHz/256 = 62.5 KHz, Pulse Time= 1/62.5 KHz = 16μs
  Speed of Timer1 = 16MHz/64 = 250kHz(clock cycle/sec), Pulse Time= 1/250kHz KHz = 4μs
  Speed of Timer1 = 16MHz/256 = 62.5 KHz, Pulse Time= 1/62.5 KHz = 16μs
*/

  // Set OC1A (Pin 9) as output == DIG9/PIN 9 in arduino and based on board
  DDRB |= (1 << DDB1);

  // Duty Cycle  (20ms period) / 1/50Hz . Duty Cycle of 5 volts = 1-2ms
  /*
  Clock cycles per second = Clock Speed / Prescaler = 16,000,000 / 64 = 250,000 cycles/second.
  Clock cycles in 20 milliseconds = 250,000 cycles/second * 0.02 seconds = 5,000 cycles. (Minus 1, since the starting value is 0)
*/
  OCR1A = 4999;  //

  // OCR1A = 375;

  // // Configure Timer 1 for input capture
  //   TCCR1A = 0; // Normal mode
  //   TCCR1B |= (1 << ICES1) | (1 << ICNC1); // Capture rising edge, noise canceler enabled
  //   TIMSK1 |= (1 << ICIE1); // Enable input capture interrupt
}

void initUltraSonic() {
  // TRIG pin (digital pin 12) as output
  DDRB |= (1 << trigPin);

  // TRIG pin CLEAR
  PORTB |= ~(1 << trigPin);

  // ECHO pin (digital pin 13) as input
  DDRB &= ~(1 << echoPin);
}

double readEcho() {

  // This function will be called when an input capture interrupt occurs
  // Measure the width of the echo pulse captured by Timer 1
  // Calculate the width in microseconds and store it in echo_pulse_width variable

  static uint16_t start_time = 0;
  static uint16_t echo_pulse_width = 0;

  while (!(PINB & (1 << echoPin)))
    ;

  TCNT1 = 0;  // Reset timer count to 0

  while ((PINB & (1 << echoPin)))
    ;

  // Calculate distance
  double duration = TCNT1;  // Read timer count
  // Convert duration to distance
  // Serial.println(duration);
  // uint16_t distance = duration * 0.068; //for cm
  double distance = duration * 0.0267716;  //for inches
  return distance;
}


void toggleLED(String position) {

  if (position == "center") {
    PORTB |= (1 << LED_LEFT);
    PORTB |= (1 << LED_RIGHT);
  } else if (position == "right") {
    PORTB &= ~(1 << LED_LEFT);
    PORTB |= (1 << LED_RIGHT);
  } else if (position == "left") {
    PORTB |= (1 << LED_LEFT);
    PORTB &= ~(1 << LED_RIGHT);
  }
}

void offLED(String position) {
  if (position == "center") {
    PORTB &= ~(1 << LED_LEFT);
    PORTB &= ~(1 << LED_RIGHT);
  } else if (position == "right") {
    PORTB &= ~(1 << LED_LEFT);
    PORTB &= ~(1 << LED_RIGHT);
  } else if (position == "left") {
    PORTB &= ~(1 << LED_LEFT);
    PORTB &= ~(1 << LED_RIGHT);
  }
}

void moveTheServo(int duty_cycle) {
  OCR1A = duty_cycle;
}

void initLED() {
  // Setup LED 10 and 11 as output
  DDRB |= (1 << LED_LEFT);   //dig10
  DDRB |= (1 << LED_RIGHT);  //dig11
  delayMicroseconds(10);
  PORTB &= ~(1 << LED_LEFT);
  PORTB &= ~(1 << LED_RIGHT);
}

void triggerUltrasonicSensor() {
  // Serial.println("Triggering UltraSonic");
  // Send a 10 microsecond trigger pulse on a digital output pin
  // Clear the trigPin
  // digitalWrite(trigPin, LOW);
  PORTB &= ~(1 << trigPin);
  delayMicroseconds(2);

  // Set the trigPin on HIGH state for 10 microseconds
  // digitalWrite(trigPin, HIGH);
  PORTB |= (1 << trigPin);
  delayMicroseconds(10);

  // digitalWrite(trigPin, LOW);
  PORTB &= ~(1 << trigPin);

  // Enable input capture interrupt
  // TCCR1B |= (1 << ICES1) | (1 << ICNC1) | (1 << ICIE1);  // Capture rising edge, noise canceler enabled, enable input capture interrupt
}


void center_To_Left() {
  //  center to  left
  for (int i = 375; i <= 625; i += 2) {  // 0 degrees: 1.5 ms pulse, 90 degrees: 2.5 ms pulse
    Serial.println(i);
    moveTheServo(i);  // Move servo to current angle
  }

  triggerUltrasonicSensor();
  distance = readEcho();
  Serial.println(distance);
  if (distance < 5) {
    toggleLED("left");
    _delay_ms(500);  // Wait for 5 ms
  } else {
    offLED("left");
  }

  _delay_ms(500);  // Wait for 5 ms
}

void left_To_Center() {

  //  left to  center
  for (int i = 625; i >= 375; i -= 2) {  // 0 degrees: 1.5 ms pulse, 90 degrees: 2.5 ms pulse
    Serial.println(i);
    moveTheServo(i);  // Move servo to current angle
  }

  triggerUltrasonicSensor();
  distance = readEcho();
  Serial.println(distance);
  if (distance < 5) {
    toggleLED("center");
    _delay_ms(500);  // Wait for 5 ms
  } else {
    offLED("center");
  }

  _delay_ms(500);  // Wait for 5 ms
}

void center_To_Right() {

  // center to right
  for (int i = 375; i >= 0; i -= 2) {  // 0 degrees: 1.5 ms pulse, 90 degrees: 2.5 ms pulse
    Serial.println(i);
    moveTheServo(i);  // Move servo to current angle
  }

  triggerUltrasonicSensor();
  distance = readEcho();
  Serial.println(distance);
  if (distance < 5) {
    toggleLED("right");
    _delay_ms(500);  // Wait for 5 ms
  } else {
    offLED("right");
  }

  _delay_ms(500);  // Wait for 5 ms
}

void right_To_Center() {

  // right to center
  for (int i = 0; i <= 375; i += 2) {  // 0 degrees: 1.5 ms pulse, 90 degrees: 2.5 ms pulse
    Serial.println(i);
    moveTheServo(i);  // Move servo to current angle
  }

  triggerUltrasonicSensor();
  distance = readEcho();
  Serial.println(distance);
  if (distance < 5) {
    toggleLED("center");
    _delay_ms(500);  // Wait for 5 ms
  } else {
    offLED("center");
  }
  _delay_ms(500);  // Wait for 5 ms
}

//fixed direction
double read_only_ULTRASONIC() {
  triggerUltrasonicSensor();
  distance = readEcho();
  Serial.println(distance);
  return distance;
}
