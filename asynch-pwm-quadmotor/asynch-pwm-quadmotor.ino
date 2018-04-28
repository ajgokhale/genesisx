const int DEFAULT_SPEED = 255;
//ROTATION LIMITERS (DUMMY)
const long minRotation = 0; 
const long maxRotation = 1000;
//MOTOR VARIANCE LIMITERS (DUMMY)
const double varThreshold = 5000;

/*Utility Functions*/
boolean sign(int value) { 
 if (value >= 0) {
  return true;
 }
 return false;
}

/*Motor Class*/
class Motor {
  private:
    int currentSpeed;
    unsigned int speedPin; //PWM pin
    unsigned int directionPin;
    unsigned int brakePin;
  public:
    Motor(unsigned int, unsigned int, 
      unsigned int, unsigned int, unsigned int);
    void runMotor(bool);
    void runMotor(int);
    void stopMotor();
    void setPins();
    unsigned int encoderPinA;
    unsigned int encoderPinB;
    bool lastAVal;
    bool lastBVal;
    long count;
    double motorSpeed; //UNIT: count per second
                       //calculated every 10 milliseconds
                       //need to optimize this number^
};

Motor::Motor(unsigned int fPin, unsigned int rPin, unsigned int bPin,
  unsigned int ePinA , unsigned int ePinB) {
  this->speedPin  = fPin;
  this->directionPin  = rPin;
  this->brakePin    = bPin;
  this->encoderPinA = ePinA;
  this->encoderPinB = ePinB;
  this->setPins();
  this->currentSpeed = 0;
  this->count = 0;
  this->lastAVal = LOW;//digitalRead(this->encoderPinA);
  this->lastBVal = LOW;//digitalRead(this->encoderPinB);
}

void Motor::setPins() {
  pinMode(this->speedPin,  OUTPUT);
  pinMode(this->directionPin,  OUTPUT);
  pinMode(this->brakePin,    OUTPUT);
  //pinMode(this->encoderPinA, INPUT);
  //pinMode(this->encoderPinB, INPUT);
  //attachInterrupt(digitalPinToInterrupt(this->encoderPinA), detected, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(this->encoderPinB), detected, CHANGE);
}

void Motor::runMotor(bool dir) {
  this->currentSpeed = (2 * dir - 1) * DEFAULT_SPEED;
  analogWrite(this->speedPin,     DEFAULT_SPEED);
  analogWrite(this->directionPin, dir);
  digitalWrite(this->brakePin, HIGH);
}

void Motor::runMotor(int mSpeed) {
  this->currentSpeed = mSpeed;
  bool dir = sign(mSpeed);
  int rSpeed = abs(mSpeed);
  analogWrite(this->speedPin,     rSpeed);
  analogWrite(this->directionPin, dir);
  digitalWrite(this->brakePin, HIGH);
}
void Motor::stopMotor() {
  digitalWrite(this->brakePin, LOW);
  this->currentSpeed = 0;
  analogWrite(this->speedPin, 0);
}

const unsigned int MOTORS_NUM = 4;
Motor* motors [MOTORS_NUM] = {};
const byte ACK = 255;
unsigned int offset;
unsigned int timestamp = 0;
byte dataRcvd = 0;

/*
 * Reset clock to zero
 * Write to serial, acknowledging clock reset
 * Processor will reset once it receives acknowledgement
 */
 
void resetClock() {
  Serial.write(ACK);
  offset = millis();
}


void runMotors(byte data) {
  int motorControl;
  for(int i = 0; i < MOTORS_NUM; i += 1) {
    motorControl = data % 3 - 1;
    if (motorControl >= 0) {
      motors[i]->runMotor(motorControl);
    }
    else {
      motors[i]->stopMotor();
    }
    data = data / 3;
  }
}

void flushBuffer(unsigned int remaining) {
  while (Serial.available() > remaining) {
    Serial.read();
  }
}
/*
void detected() {
  for(int i = 0; i < MOTORS_NUM; i += 1) {
    bool aVal = digitalRead(motors[i]->encoderPinA);
    bool bVal = digitalRead(motors[i]->encoderPinB);
    bool plus  = aVal ^ motors[i]->lastBVal;
    bool minus = bVal ^ motors[i]->lastAVal;
    if (plus) {
      motors[i]->count += 1;
    }
    if (minus) {
      motors[i]->count -= 1;
    }
    motors[i]->lastAVal = aVal;
    motors[i]->lastBVal = bVal;
  }
}
*/
ISR (PCINT0_vect) {
  for(int i = 0; i < MOTORS_NUM; i += 1) {
    bool aVal = digitalRead(motors[i]->encoderPinA);
    bool bVal = digitalRead(motors[i]->encoderPinB);
    bool plus  = aVal ^ motors[i]->lastBVal;
    bool minus = bVal ^ motors[i]->lastAVal;
    if (plus) {
      motors[i]->count += 1;
    }
    if (minus) {
      motors[i]->count -= 1;
    }
    motors[i]->lastAVal = aVal;
    motors[i]->lastBVal = bVal;
  }
}

ISR (PCINT2_vect) {
  for(int i = 0; i < MOTORS_NUM; i += 1) {
    bool aVal = digitalRead(motors[i]->encoderPinA);
    bool bVal = digitalRead(motors[i]->encoderPinB);
    bool plus  = aVal ^ motors[i]->lastBVal;
    bool minus = bVal ^ motors[i]->lastAVal;
    if (plus) {
      motors[i]->count += 1;
    }
    if (minus) {
      motors[i]->count -= 1;
    }
    motors[i]->lastAVal = aVal;
    motors[i]->lastBVal = bVal;
  }
}

void pciSetup(byte pin) {
  *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));
  PCIFR |= bit(digitalPinToPCICRbit(pin));
  PCICR |= bit(digitalPinToPCICRbit(pin));
}
long tme = millis();
boolean state = false;
void setup() {
  Serial.begin(9600);
  
  pciSetup(2); pciSetup(4);  pciSetup(6);  pciSetup(7);
  pciSetup(8); pciSetup(10); pciSetup(12); pciSetup(13);
  
  Motor motor0(3, 14, 15, 2, 4);
  motors[0] = &motor0;
  
  Motor motor1(5, 14, 15, 12, 13);
  motors[1] = &motor1;
  
  Motor motor2(9, 14, 15, 8, 10);
  motors[2] = &motor2;
  
  Motor motor3(11, 16, 17, 6, 7);
  motors[3] = &motor3;
  //motors[2]->runMotor(true);
  //motors[3]->runMotor(true);
  //motors[3]->stopMotor();

}

boolean checkRange() {
  for(int i = 0; i < MOTORS_NUM; i += 1) {
    if (motors[i]->count < minRotation ||
        motors[i]->count > maxRotation) {
      return false;
    }
  }
  return true;
}

boolean checkConsistency() {
  long total = 0;
  for(int i = 0; i < MOTORS_NUM; i += 1) {
    total += motors[i]->count;
  }
  double average = (double) total / (double) MOTORS_NUM;
  double totalVar = 0;
  for(int i = 0; i < MOTORS_NUM; i += 1) {
    totalVar += sq(motors[i]->count - average);
  }
  double variance = totalVar / (double) MOTORS_NUM;
  if (variance > varThreshold) {
    return false;
  }
  else {
    return true;
  }
}

void loop() {
  if (millis() - tme > 1000) {
    Serial.println(motors[3]->count);
    Serial.println(motors[2]->count);
    Serial.println(motors[1]->count);
    Serial.println(motors[0]->count);
    tme = millis();
  }
/*
  if (!checkRange() ||
      !checkConsistency()) {
    for(int i = 0; i < MOTORS_NUM; i += 1) {
      motors[i]->stopMotor();
    }    
  }
*/    
}

