from collections import deque
import matplotlib.pyplot as plt
import numpy as np
import threading

import time
import datetime

# blocking = 1
plt.ion() ## Note this correction
fig=plt.figure()
plt.axis([0,1000,0,1])


exitFlag = 0

a1 = deque([float(0)]*100) # append & pop from opposite ends is thread safe according to .py doc 

sema = threading.Condition()

def add_plot_data(new_roll_data, new_pitch_data, new_yaw_data):
    sema.acquire(blocking = 1)
    a1.appendleft(new_roll_data)
    datatoplot = a1.pop() # only want 100 values to plot at any one time    
    sema.notify() # notify plot that data is available
    sema.release()
    
class PlotThread(threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name

        # init plot control
        self.ax = plt.axes(xlim=(0, 20), ylim=(-10, 10))
        self.line, = plt.plot(a1)
                
    def run(self):
        print "starting " + self.name
        self.simplePlot()
        print "Exiting " + self.name

    def simplePlot(self):

        plt.ion() # interactive mode on
        plt.grid()
        
        while 1:
            sema.acquire(blocking = 1)
            sema.wait() # wait for data
            self.line.set_ydata(a1)
            sema.release()

            plt.draw()
#            print "%s| %s" % (datetime.datetime.now().isoformat(), a1[0])
        


def runPlot():
    cplot = PlotThread(42, "2D-plot-thread")
    cplot.start() # calls the "run" method above


def myTimeStamp():
    print datetime.datetime.now().isoformat()

    

