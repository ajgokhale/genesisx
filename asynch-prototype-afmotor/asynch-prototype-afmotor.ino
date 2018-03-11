#include <AFMotor.h>
const int DEFAULT_SPEED = 200;
/* 
 * Utility Functins 
 */
boolean sign(int value) { 
 if (value >= 0) {
  return true;
 }
 return false;
}

const unsigned int MOTORS_NUM = 2;
AF_DCMotor* motors[MOTORS_NUM] = {};
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
    switch (motorControl) {
      case -1:
        motors[i]->run(BACKWARD);
        break;
      case 1 :
        motors[i]->run(FORWARD);
        break;
      default:
        motors[i]->run(RELEASE);
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
  for (int i = 0; i < MOTORS_NUM; i++) {
    motors[i] = &AF_DCMotor(i + 1, MOTOR12_1KHZ);
    motors[i]->setSpeed(DEFAULT_SPEED);
  }
  offset = 0;
}

void loop() {
  //FOR TESTING//
  Serial.write(255);
  ///////////////
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

 

