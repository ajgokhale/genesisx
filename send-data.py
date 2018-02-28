import serial
import time
import csv
import struct

import time,os,string,math
import wave

PORT = 'COM4'
try:
    ser = serial.Serial(PORT, 9600, timeout = 0)
except:
    raise Exception('Cannot connect')

def read_from_csv():
    r = csv.reader(open('dummy-data.csv')) 
    lines = []
    for row in r:
        for code in row:
            lines.append(int(code))
    return lines

waitingForUser = True
readingData = False
time_to_open = 10
num_of_samples = 5

def packIntegerAsULong(value):
    """Packs a python 4 byte unsigned integer to an arduino unsigned long"""
    return struct.pack('I', value)    #should check bounds

def writeData(serialPort):
    serialPort.flushInput()
    serialPort.flushOutput()
    serialPort.write(val)

def readData(port, delay):
    current_time = time.clock()
    while(time.clock() < current_time + delay):
        writeData(port)
        time.sleep(0.1)
        try:
            data = port.readline()
        except port.SerialTimeoutException:
            print('Lost connection')
            continue
        if data: #checks if any data was read
            return data
    return False

def resetClock():
    



data_count = 0
us_data = [0]*num_of_samples


dummy_data = read_from_csv()
while True:
    for code in dummy_data:
        writeData()


'''
while True:
    val = packIntegerAsULong(state+1)
    data_rcvd = False
    while(not data_rcvd):
        print("Receiving data...")
        #print(state, val, data_rcvd)
        data_rcvd = readData(ser, 10)
    data= data_rcvd.decode("utf-8").strip()
    if state == waitingForUser:
        data_count = 0
        user = data
        old_frame = f2
        f2 = second_screen(user)
        old_frame.destroy()
        state = readingData
        val = packIntegerAsULong(state+1)
        data_rcvd = False
        while(not data_rcvd or data_rcvd != state+1):
            print("Receiving lock data...1")
            data_rcvd = readData(lock_ser, 10)
            data_rcvd = int(data_rcvd.decode("utf-8").strip())
            print(data_rcvd)
        time_unlocked = time.clock()
    else:
        door_state,dataList = convertDataToList(user, data)
        us_data[data_count] = dataList[1]
        data_count += 1
        if data_count == num_of_samples:
            dataList[1] = max(set(us_data), key=us_data.count)
            data_count = 0
            if not editor(dataList):
                print('Could not save to CSV')
        time_since_unlock = time.clock() - time_unlocked
        if time_since_unlock > time_to_open and not door_state:
            #take_picture()
            state = waitingForUser
            raise_frame(f1)
            data_rcvd = False
            val = packIntegerAsULong(state+1)
            while(not data_rcvd or data_rcvd != state+1):
                print("Receiving lock data...")
                data_rcvd = readData(lock_ser, 10)
                data_rcvd = int(data_rcvd.decode("utf-8").strip())
                print(data_rcvd)
        
        

    time.sleep(1)
'''