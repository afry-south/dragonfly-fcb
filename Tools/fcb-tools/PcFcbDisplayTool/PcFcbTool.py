# borrowed ideas from miniterm.py, part of Python2.7 distribution.

import sys
import time
import os
import serial
import threading
from serial.tools.list_ports import comports
import dragonfly_fcb_pb2 
from __builtin__ import str


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

    while 1:
        line = ""
        data = ' '
        while data != '\n':
            data = mySerial.read(1);
            line += str(data)
        
        # CONTINUE HERE
        # http://stackoverflow.com/questions/2293780/how-to-detect-a-floating-point-number-using-a-regular-expression
        sys.stdout.write(line);
    print "exiting while loop"

interval_ms = 1000
duration_s = 20

# use "import optparse" in future - see miniterm.py in Python installation for example.
# this will allow sophisticaed CLI args parsing
mySerial = serial.serial_for_url("\\.\COM8", 115200, parity='N', rtscts=False, xonxoff=False, timeout=1)
myComReaderThread = threading.Thread(target=comReader, name="tComReader")
myComReaderThread.daemon = True
myComReaderThread.start()
mySerial.write("about\n".encode())
time.sleep(2)
mySerial.write("start-state-sampling %d %d c" % (interval_ms, duration_s))
myComReaderThread.join(duration_s + 1)
