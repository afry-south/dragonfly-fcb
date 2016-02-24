# borrowed ideas from miniterm.py, part of Python2.7 distribution.

import sys
import time
import os
import serial
import threading
from serial.tools.list_ports import comports
import dragonfly_fcb_pb2
import PcFcbDisplayPlot
import google.protobuf.message
from __builtin__ import str, int


print "imports successful, Captain!"

print "OS name:" + os.name

mySerial = 0;

# NOTE:
# this function is only semi-useful because it doesn't print
# the correct format of the device address to respective port
def ask_for_port():
    """\
    Show a list of ports and ask the user for a choice. To make selection
    easier on systems with long device names, also allow the input of an
    index.
    """
    sys.stderr.write('\n--- Available ports:\n')
    ports = []
    for n, (port, desc, hwid) in enumerate(sorted(comports()), 1):
        #~ sys.stderr.write('--- %-20s %s [%s]\n' % (port, desc, hwid))
        sys.stderr.write('--- {:2}: {:20} {}\n'.format(n, port, desc))
        ports.append(port)
    while True:
        port = raw_input('--- Enter port index or full name: ')
        try:
            index = int(port) - 1
            if not 0 <= index < len(ports):
                sys.stderr.write('--- Invalid index!\n')
                continue
        except ValueError:
            pass
        else:
            port = ports[index]
        return port

# ask_for_port()

def comReader():
    print "comReader ready, Captain!!"
    print "comReader is executing in: " + threading.currentThread().name
    state = dragonfly_fcb_pb2.FlightStatesProto()

    while 1:
        line = ""
        data = ' '
        while (data != '\r'):
            data = mySerial.read(1);
            if (data != '\n') and (data != '\r'):
                line += str(data)

        hexified = ':'.join(x.encode('hex') for x in line)
        if hexified[:2] == "04":
            try:
                state.ParseFromString(line[4:]);
            except google.protobuf.message.DecodeError as de:
                print de
                continue
            # sys.stdout.write(state.__str__());
            PcFcbDisplayPlot.add_plot_data(state.rollAngle, state.pitchAngle, state.yawAngle)
        else:
            pass
            # print line

        
    print "exiting while loop"

interval_ms = 500
duration_s = 20

# use "import optparse" in future - see miniterm.py in Python installation for example.
# this will allow sophisticaed CLI args parsing
mySerial = serial.serial_for_url("\\.\COM8", 115200, parity='N', rtscts=False, xonxoff=False, timeout=1)
myComReaderThread = threading.Thread(target=comReader, name="tComReader")
myComReaderThread.daemon = True
myComReaderThread.start()
mySerial.write("about\n".encode()) # todo
PcFcbDisplayPlot.runPlot()
time.sleep(1)
mySerial.write("start-state-sampling %d %d p" % (interval_ms, duration_s))
myComReaderThread.join(duration_s + 1)
