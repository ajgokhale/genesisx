const int DEFAULT_SPEED = 255;

/* 
 * Utility Functins 
 */
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
    unsigned int forwardPin; //PWM pin
    unsigned int reversePin; //PWM pin
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
    int count;
};

Motor::Motor(unsigned int fPin, unsigned int rPin, unsigned int bPin,
  unsigned int ePinA , unsigned int ePinB) {
  this->forwardPin  = fPin;
  this->reversePin  = rPin;
  this->brakePin    = bPin;
  this->encoderPinA = ePinA;
  this->encoderPinB = ePinB;
  this->setPins();
  this->currentSpeed = 0;
  this->count = 0;
  this->lastAVal = digitalRead(this->encoderPinA);
  this->lastBVal = digitalRead(this->encoderPinB);
}

void Motor::setPins() {
  pinMode(this->forwardPin,  OUTPUT);
  pinMode(this->reversePin,  OUTPUT);
  pinMode(this->brakePin,    OUTPUT);
//pinMode(encoderPinA, INPUT);
//pinMode(encoderPinB, INPUT);
  attachInterrupt(this->encoderPinA, detected, RISING);
  attachInterrupt(this->encoderPinB, detected, RISING);
}

void Motor::runMotor(bool dir) {
  this->currentSpeed = (2 * dir - 1) * DEFAULT_SPEED;
  analogWrite(this->forwardPin,   dir  * DEFAULT_SPEED);
  analogWrite(this->reversePin, (!dir) * DEFAULT_SPEED);
  digitalWrite(this->brakePin, HIGH);
}

void Motor::runMotor(int mSpeed) {
  this->currentSpeed = mSpeed;
  bool dir = sign(mSpeed);
  int rSpeed = abs(mSpeed);
  analogWrite(this->forwardPin,   dir  * rSpeed);
  analogWrite(this->reversePin, (!dir) * rSpeed);
  digitalWrite(this->brakePin, HIGH);
}
void Motor::stopMotor() {
  digitalWrite(this->brakePin, LOW);
  this->currentSpeed = 0;
  analogWrite(this->forwardPin, 0);
  analogWrite(this->reversePin, 0);
}

const unsigned int MOTORS_NUM = 3;
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

void setup() {
  Serial.begin(9600);
  motors[0] = &Motor(9,  6,  2,  A0, A1);
  motors[1] = &Motor(5,  3,  4,  A2, A3);
//motors[2] = &Motor(11, 10, 12, A4/*, A5*/);
}

void loop() {
  motors[0]->runMotor(true);
}

