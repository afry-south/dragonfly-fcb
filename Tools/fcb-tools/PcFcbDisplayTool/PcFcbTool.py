# borrowed ideas from miniterm.py, part of Python2.7 distribution.

import sys
import os
import time
import serial
import threading
from serial.tools.list_ports import comports
import dragonfly_fcb_pb2
import google.protobuf.message
from collections import deque

from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg

debug_print = False
script_name = os.path.basename(__file__)[:-3]

def pprint(format, *args):
    string_to_print = script_name + ": " + format % args
    print string_to_print

def dprint(format, *args):
    string_to_print = script_name + ": " + format % args
    if debug_print:
        pprint(string_to_print)
        
pprint("starting ...")

dprint("imports successful, Captain!")

fcb_serial = 0;
do_exit = False;
sema = threading.Condition()

roll_data = deque([float(0)]*100) # append & pop from opposite ends is thread safe according to .py doc 
pitch_data = deque([float(0)]*100) # append & pop from opposite ends is thread safe according to .py doc 
yaw_data = deque([float(0)]*100) # append & pop from opposite ends is thread safe according to .py doc 

def update_plot_data(new_roll_data, new_pitch_data, new_yaw_data):
    sema.acquire(blocking = 1)
    roll_data.appendleft(new_roll_data)
    datatoplot = roll_data.pop() # only want 100 values to plot at any one time
    pitch_data.appendleft(new_pitch_data)
    datatoplot = pitch_data.pop() # only want 100 values to plot at any one time    
    yaw_data.appendleft(new_yaw_data)
    datatoplot = yaw_data.pop() # only want 100 values to plot at any one time    
    
    sema.notify() # notify plot that data is available
    sema.release()

# handy tip, type: python -m serial.tools.list_ports
# to list available COM ports at the terminal.

def comReader(sample_nbr):
    global do_exit
    pprint("%s starting" % comReader.__name__)
    dprint("comReader is executing in: " + threading.currentThread().name)
    state = dragonfly_fcb_pb2.FlightStatesProto()
    i = 0;

    while (i < sample_nbr and not do_exit):
        line = ""
        data = ' '
        while (data != '\r'):
            try:
                data = fcb_serial.read(1);
            except serial.SerialException as se:
                pprint("serial.SerialException: %s" % (se.__str__()))
                do_exit = True
                break
            if (data != '\n') and (data != '\r'):
                line += str(data)
        
        if do_exit:
            break
        
        i += 1;
        hexified = ':'.join(x.encode('hex') for x in line)
        if hexified[:2] == "04":
            try:
                state.ParseFromString(line[4:]);
            except google.protobuf.message.DecodeError as de:
                pprint("sample %d discarded: %s" % (i, de.__str__()))
                continue
            # sys.stdout.write(state.__str__()); # useful for debugging
            update_plot_data(state.rollAngle, state.pitchAngle, state.yawAngle)
        else:
            pprint("%s received: %s", comReader.__name__, line)

    fcb_serial.close()
    pprint("%s done - close graphics window to exit" % comReader.__name__)


#### Graphics objects & helper functions

app = QtGui.QApplication([])

win = pg.GraphicsWindow(title="Basic plotting examples")
win.resize(1000,600)
win.setWindowTitle('pyqtgraph example: Plotting')
pg.setConfigOptions(antialias=True) # Enable antialiasing for prettier plots

roll_plot = win.addPlot(title="Real-time ish roll angle[rad] plot")
roll_plot_curve = roll_plot.plot(pen='r', name='Roll[rad]')
roll_plot.showGrid(x=True, y=True)
win.nextRow()
pitch_plot = win.addPlot(title="Real-time ish pitch angle[rad] plot")
pitch_plot_curve = pitch_plot.plot(pen='g', name='Pitch[rad]')
pitch_plot.showGrid(x=True, y=True)
win.nextRow()
yaw_plot = win.addPlot(title="Real-time ish yaw angle[rad] plot")
yaw_plot_curve = yaw_plot.plot(pen='y', name='Yaw[rad]')
yaw_plot.showGrid(x=True, y=True)

def plotUpdate():
    global ptr, roll_plot, pitch_plot, yaw_plot
    roll_plot_curve.setData(roll_data)
    pitch_plot_curve.setData(pitch_data)
    yaw_plot_curve.setData(yaw_data)


timer = QtCore.QTimer()
timer.timeout.connect(plotUpdate)
timer.start(25)

# use "import argparse" in future - this will allow sophisticaed CLI args parsing

interval_ms = 100
duration_s = 20

interval_s = interval_ms/float(1000)
sample_nbr = int(float(duration_s / interval_s))

# use "import optparse" in future - see miniterm.py in Python installation for example.
# this will allow sophisticaed CLI args parsing
fcb_serial = serial.Serial("\\.\COM8", 115200, parity=serial.PARITY_NONE, rtscts=False, xonxoff=False, timeout=1, write_timeout=1)
myComReaderThread = threading.Thread(target=comReader, name="tComReader", args=(sample_nbr,))
myComReaderThread.daemon = True
myComReaderThread.start()
fcb_serial.write("about\n") # data does not show up when removing this line ...
time.sleep(1)

try:
    fcb_serial.write("start-state-sampling %d %d p" % (interval_ms, duration_s))
except serial.SerialTimeoutException as ste:
    fcb_serial.close()
    pprint(ste.__str__())
    quit()

if __name__ == '__main__':
    ## Start Qt event loop unless running in interactive mode or using pyside.
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        pprint("close window to exit program")
        QtGui.QApplication.instance().exec_()
        do_exit = True

myComReaderThread.join(duration_s + 1)
if myComReaderThread.isAlive():
    do_exit = True
    fcb_serial.close()
    time.sleep(interval_s)
    
pprint("... please wait for program to exit")
if fcb_serial.is_open:
    fcb_serial.close()