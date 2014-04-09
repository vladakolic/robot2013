#!/usr/bin/env python
import abc
import threading
import Tkinter as tk

import roslib; roslib.load_manifest('rospy_easy_gui')
import rospy

from std_msgs.msg import String
from motor_control.msg import PIDGains

# global flag for shutting down the rosnode after the gui is closed
applicationIsClosed = False
proportionalGain = 0.0
integralGain = 0.0
derivationGain = 0.0
sendNewMessage = False

class Observer():
    __metaclass__ = abc.ABCMeta
    def update(self):
        return

class Observable():
    def __init__(self):
        self._observers = []
        self._data = None

    def set(self, data):
        self._data = data
        for o in self._observers:
            o.update(self.data)

    def get(self):
        return self._data

#
# GUI class
#
class Application(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.grid()
        self.createWidgets()

    def createWidgets(self):
        # controls for proportional gain
        self.proportionalLabel = tk.Label(self, text='P')
        self.proportionalLabel.grid(row=0, column=0)
        self.proportionalSpinbox = tk.Spinbox(self, from_=0.0, to=100.0, increment=0.1)
        self.proportionalSpinbox.grid(row=0, column=1)

        # controls for integral gain
        self.integralLabel = tk.Label(self, text='I')
        self.integralLabel.grid(row=1, column=0)
        self.integralSpinbox = tk.Spinbox(self, from_=0.0, to=100.0, increment=0.1)
        self.integralSpinbox.grid(row=1, column=1)
        
        # controls for derivation gain
        self.derivationLabel = tk.Label(self, text='D')
        self.derivationLabel.grid(row=2, column=0)
        self.derivationSpinbox = tk.Spinbox(self, from_=0.0, to=100.0, increment=0.1)
        self.derivationSpinbox.grid(row=2, column=1)
        
        self.sendButton = tk.Button(self, text='apply gains', command=self.sendMessage)
        self.sendButton.grid(row=3, column=0, columnspan=2)

    def sendMessage(self):
        global proportionalGain
        global integralGain
        global derivationGain
        global sendNewMessage

        #print "P: %g I: %g D: %g" % (float(self.proportionalSpinbox.get()), float(self.integralSpinbox.get()), float(self.derivationSpinbox.get()))
        proportionalGain = float(self.proportionalSpinbox.get())
        integralGain = float(self.integralSpinbox.get())
        derivationGain = float(self.derivationSpinbox.get())
        sendNewMessage = True

    def signalizeQuit(self):
        global applicationIsClosed
        applicationIsClosed = True
        self.quit()

    def changeOption(self):
        global option
        option = not option

class ApplicationWrapper(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
    
    def run(self):
        self.app = Application()
        self.app.master.title = 'Application'
        self.app.mainloop()

#
#
#
def talker():
    global sendNewMessage

    pub = rospy.Publisher('/motor_control/PIDGains', PIDGains)
    rospy.init_node('rospy_easy_gui')

    while not rospy.is_shutdown() and not applicationIsClosed:

        if sendNewMessage:
            msg = PIDGains()
            msg.fP = proportionalGain
            msg.fI = integralGain
            msg.fD = derivationGain
            pub.publish(msg)
            sendNewMessage = False

        rospy.sleep(1.0)


#
# MAIN
#

threadPool = []

if __name__ == '__main__':
    # add threaded application classes
    app = ApplicationWrapper()
    app.start()

    try:
        talker()
    except rospy.ROSInterruptException:
        pass

    app.join()


