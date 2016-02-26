# borrowed ideas from miniterm.py, part of Python2.7 distribution.

import sys
import time
import os
import serial
import threading
from serial.tools.list_ports import comports
import dragonfly_fcb_pb2
import google.protobuf.message
from collections import deque

print "imports successful, Captain!"

print "OS name:" + os.name

mySerial = 0;
doExit = False;
sema = threading.Condition()

roll_data = deque([float(0)]*100) # append & pop from opposite ends is thread safe according to .py doc 
pitch_data = deque([float(0)]*100) # append & pop from opposite ends is thread safe according to .py doc 
yaw_data = deque([float(0)]*100) # append & pop from opposite ends is thread safe according to .py doc 

def update_plot_data(new_roll_data, new_pitch_data, new_yaw_data):
    sema.acquire(blocking = 1)
    roll_data.appendleft(new_roll_data)
    datatoplot = roll_data.pop() # only want 100 values to plot at any one time
    pitch_data.appendleft(new_roll_data)
    datatoplot = pitch_data.pop() # only want 100 values to plot at any one time    
    roll_data.appendleft(new_yaw_data)
    datatoplot = yaw_data.pop() # only want 100 values to plot at any one time    
    
    sema.notify() # notify plot that data is available
    sema.release()

# handy tip, type: python -m serial.tools.list_ports
# to list available COM ports at the terminal.

def comReader(sample_nbr):
    global doExit
    print "comReader ready, Captain!!"
    print "comReader is executing in: " + threading.currentThread().name
    state = dragonfly_fcb_pb2.FlightStatesProto()
    i = 0;

    while (i < sample_nbr and not doExit):
        line = ""
        data = ' '
        while (data != '\r'):
            try:
                data = mySerial.read(1);
            except serial.SerialException as se:
                print se
                doExit = True
                break
            if (data != '\n') and (data != '\r'):
                line += str(data)

        hexified = ':'.join(x.encode('hex') for x in line)
        if hexified[:2] == "04":
            try:
                state.ParseFromString(line[4:]);
            except google.protobuf.message.DecodeError as de:
                print de
                continue
            sys.stdout.write(state.__str__());
            update_plot_data(state.rollAngle, state.pitchAngle, state.yawAngle)
            i += 1;
            print "sample i:%d" % (i)
        else:
            print line
            pass
                
            # print line
    mySerial.close()
    print "exiting comReader loop"

# use "import optparse" in future - see miniterm.py in Python installation for example.
# this will allow sophisticaed CLI args parsing

interval_ms = 500
duration_s = 5

interval_s = interval_ms/float(1000)
sample_nbr = int(float(duration_s / interval_s))

# use "import optparse" in future - see miniterm.py in Python installation for example.
# this will allow sophisticaed CLI args parsing
mySerial = serial.Serial("\\.\COM8", 115200, parity=serial.PARITY_NONE, rtscts=False, xonxoff=False, timeout=1, write_timeout=1)
myComReaderThread = threading.Thread(target=comReader, name="tComReader", args=(sample_nbr,))
myComReaderThread.daemon = True
myComReaderThread.start()
mySerial.write("about\n") # data does not show up when removing this line ...
time.sleep(1)
try:
    mySerial.write("start-state-sampling %d %d p" % (interval_ms, duration_s))
except serial.SerialTimeoutException as ste:
    mySerial.close()
    print ste
    quit()
    
myComReaderThread.join(duration_s + 1)
if myComReaderThread.isAlive():
    doExit = True
    mySerial.close()
    time.sleep(interval_s)
    
print "after join"
if mySerial.is_open:
    mySerial.close()