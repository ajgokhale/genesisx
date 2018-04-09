#include <PololuWheelEncoders.h>

volatile byte half_revolutions;
unsigned int rpm;
unsigned long timeold;

#define m1a 0
#define m1b 1
#define m2a 2
#define m2b 3
#define m3a 4
#define m3b 5
#define m4a 6
#define m4b 7

PololuWheelEncoders enc12;
PololuWheelEncoders enc34;

void setup() {
  Serial.begin(9600);
  enc12.init(m1a, m1b, m2a, m2b);
  enc34.init(m3a, m3b, m4a, m4b);
}

void loop() {
  enc12.getCountsM1();
}
