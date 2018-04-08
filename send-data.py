import serial
import time
import csv
import struct
import os,string,math
import wave

FILE_NAME = 'dummy-data.csv'
ACK = 255
PORT = 'COM4' 
# '/dev/ttyUSB0'

def millis():
    return int(round(time.time() * 1000))

OFFSET = millis()

def readFromCSV():
    r = csv.reader(open(FILE_NAME))
    codes = []
    for row in r:
        for code in row:
            codes.append(int(code,3))
    return codes
    

'''
def packIntegerAsULong(value):
    """Packs a python 4 byte unsigned integer to an arduino unsigned long"""
    return struct.pack('I', value)    #should check bounds
'''

def sendData(port, code):
    timestamp = int(round((millis() - OFFSET)/ 10.0)) % 3000
    time1 = timestamp // (2**6)
    time2 = timestamp % (2**6) + 64
    data  = code               + 128
    packet = [time1, time2, data]
    # packet = [data]
    writeData(port, packet)
    '''
    writeData(port, time1)
    writeData(port, time2)
    writeData(port, data)
    '''

def writeData(port, val):
    port.write(val)

def resetClock(port):
    writeData(ACK)
    while (ser.read() != ACK):
        continue
    OFFSET = millis()
    port.reset_input_buffer()

def connectToSerial():
    try:
        return serial.Serial(PORT, 9600, timeout = 0)
    except:
        raise Exception('Cannot connect')

def printRead(ser):
    print(int.from_bytes(ser.read(), byteorder = 'big'), end='')

def main():
    ser = connectToSerial()
    codes = readFromCSV()
    while True:
        for code in codes:
            sendData(ser, code)
            time.sleep(1.5)
            for i in range(4):
                printRead(ser)
            print()


main()
