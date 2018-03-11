#include <AFMotor.h>
const int DEFAULT_SPEED = 255;
/* 
 * Utility Functins 
 */

const unsigned int MOTORS_NUM = 4;
AF_DCMotor* motors[MOTORS_NUM] = {};

void setup() {
  Serial.begin(9600);
  for (int i = 2; i < MOTORS_NUM; i++) {
    motors[i] = &AF_DCMotor(i + 1);
    motors[i]->setSpeed(DEFAULT_SPEED);
  }
}

void loop() {
  for (int i = 2; i < MOTORS_NUM; i++) {
    motors[i]->run(FORWARD);
    delay(1000);
    motors[i]->run(BACKWARD);
    delay(1000);
    motors[i]->run(RELEASE);
    delay(1000);
  }
} 
