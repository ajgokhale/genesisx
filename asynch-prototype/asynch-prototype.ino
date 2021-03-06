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
  public:
    Motor(unsigned int, unsigned int, unsigned int);
    void runMotor(bool);
    void runMotor(int);
    void stopMotor();
    unsigned int directionPin;
    unsigned int speedPin;
    unsigned int brakePin;
    void setPins();
};

Motor::Motor(unsigned int dPin, unsigned int sPin, unsigned int bPin) {
  this->directionPin = dPin;
  this->speedPin = sPin;
  this->brakePin = bPin;
  this->setPins();
  this->currentSpeed = 0;
}

void Motor::setPins() {
  pinMode(directionPin, OUTPUT);
  pinMode(brakePin, OUTPUT);
}

void Motor::runMotor(bool dir) {
  digitalWrite(this->directionPin, dir);
  digitalWrite(this->brakePin, LOW);
  analogWrite(this->speedPin, DEFAULT_SPEED);
  this->currentSpeed = 2*dir*DEFAULT_SPEED-DEFAULT_SPEED;
}

void Motor::runMotor(int mSpeed) {
  digitalWrite(this->directionPin, sign(mSpeed));
  digitalWrite(this->brakePin, LOW);
  analogWrite(this->speedPin, abs(mSpeed));
  this->currentSpeed = mSpeed;
}
void Motor::stopMotor() {
  digitalWrite(this->brakePin, HIGH);
  analogWrite(this->speedPin, 0);
}

const unsigned int MOTORS_NUM = 2;
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

void setup() {
  Serial.begin(9600);
  motors[0] = &Motor(12,3,9);
  motors[1] = &Motor(13,11,8);
  offset    = 0;
}

void loop() {
  if (Serial.available() == 0) {
    return;
  }
  else {
    flushBuffer(8);
  }
  byte packet = Serial.read();
  Serial.write(packet);
  if (packet == ACK) {
    resetClock();
    dataRcvd  = 0;
    timestamp = 0;
    return;
  }
  byte code = packet >> 6;
  packet = bitClear(packet, 7);
  if (dataRcvd == 2 && code >= 2) {
    runMotors(packet);
    Serial.write(255);
  }
  else if (code == dataRcvd) {
    if (code == 0) {
      dataRcvd  = 0;
      timestamp = 0;
    }
    packet    = bitClear(packet, 6);
    Serial.write(packet);
    timestamp =  timestamp << 6;
    timestamp += packet;
    dataRcvd  += 1;
    return;
  }
  dataRcvd  = 0;
  timestamp = 0;
}

/* NOTES
 * FLAG BYTE SCHEME
 * Data should be sent with timestamp
 * This ensures that old data can be thrown away
 * SCHEME: encode timestamp to two bytes
 * Could throw away year, date, etc.
 * (We don't need full timestamp)
 * Just keep seconds, centiseconds
 * (If the system is a minute off, there's a larger issue)
 * Also need a FLAG byte to indicate start of each packet
 * FLAG: 10000000
 * Timestamp and data will be given 7 bits (first bit -> 0)
 * This will allow for easy distinguishing of FLAG
 * 
 * FLAG BITS SCHEME
 * First two bits of every byte in a packet will encode the packet
 * Example: (01xxxxxx01xxxxxx01xxxxxx) represents a packet
 * Each packet will increment the code of the previous packet modulo 4
 * If there is any discrepancy, throw out the packet (there was likely an interruption)
 * There is a chance that the xth and (x+4)th packet could be received together
 * Probability of this is probably low
 * Code "11111111" will be a special code for updating time to zero
 * 
 * FLAG BITS SCHEME (2)
 * Same as previous scheme, except each type of packet is encoded differently
 * Timestamp (1): 00, Timestamp (2): 01, Data: 10 OR 11
 */

 
