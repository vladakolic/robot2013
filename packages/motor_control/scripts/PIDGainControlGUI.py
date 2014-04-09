#!/usr/bin/env python
import sys
import abc
import threading
import Tkinter as tk

import re

import roslib; roslib.load_manifest('rospy_easy_gui')
import rospy

from std_msgs.msg import String
from motor_control.msg import PIDGains
from motor_control.srv import PIDGainControl

# global flag for shutting down the rosnode after the gui is closed
applicationIsClosed = False

# concrete service that will be used by the GUI (set with command line argument 1)
my_service = ''

updateGains = None

app = None

class Application(tk.Frame):
    '''
    create a gui with spinboxes for every controller gain and an apply button
    '''

    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.grid()
        self.createWidgets()

    def createWidgets(self):
        # controls for proportional gain
        self.KPLabel = tk.Label(self, text='P')
        self.KPLabel.grid(row=0, column=0)
        self.KPSpinbox = tk.Spinbox(self, from_=0.0, to=100.0, increment=0.1, values=(5.))
        self.KPSpinbox.grid(row=0, column=1)

        # controls for integral gain
        self.KILabel = tk.Label(self, text='I')
        self.KILabel.grid(row=1, column=0)
        self.KISpinbox = tk.Spinbox(self, from_=0.0, to=100.0, increment=0.1)
        self.KISpinbox.grid(row=1, column=1)
        
        # controls for derivation gain
        self.KDLabel = tk.Label(self, text='D')
        self.KDLabel.grid(row=2, column=0)
        self.KDSpinbox = tk.Spinbox(self, from_=0.0, to=100.0, increment=0.1)
        self.KDSpinbox.grid(row=2, column=1)
        
        self.applyButton = tk.Button(self, text='apply gains', command=self.applyGains)
        self.applyButton.grid(row=3, column=0, columnspan=2)

    def applyGains(self):
        global KP
        global KI
        global KD
        global updateGains 

        KP = float(self.KPSpinbox.get())
        KI = float(self.KISpinbox.get())
        KD = float(self.KDSpinbox.get())
        updateGains = True


    def signalizeQuit(self):
        global applicationIsClosed
        applicationIsClosed = True
        self.quit()


class ApplicationWrapper(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
    
    def run(self):
        global app
        global my_service

        app = Application()
        app.master.title = my_service
        app.mainloop()

def ros_node():
    '''
    create a ros node that uses a service
    '''
    global my_service
    global updateGains

    # build unique node name
    rospy.init_node(re.sub('[/]', '_', my_service)[1:])

    while not rospy.is_shutdown() and not applicationIsClosed:

        if updateGains:
            rospy.wait_for_service(my_service)
            try:
                control_speed_left_gain = rospy.ServiceProxy(my_service, PIDGainControl)
                res = control_speed_left_gain(KP, KI, KD)
                print "response: KP=%g KI=%g KD=%g" % (res.fKP, res.fKI, res.fKD)
                app.KPSpinbox.delete(0,"end")
                app.KPSpinbox.insert(0, res.fKP)
                app.KISpinbox.delete(0,"end")
                app.KISpinbox.insert(0, res.fKI)
                app.KDSpinbox.delete(0,"end")
                app.KDSpinbox.insert(0, res.fKD)
            except rospy.ServiceException, error:
                print "Service call failed: %s" % error

            updateGains = False
            
        rospy.sleep(0.1)



if __name__ == '__main__':
    '''
    create application and ros_node in different threads
    '''

    if len(sys.argv) < 2:
        print "invalid number of arguments %d\nusage: %s <service name>\n" % (len(sys.argv), sys.argv[0])
        sys.exit(1)

    # define service 
    my_service = str(sys.argv[1])
    print "service name: %s" % my_service

    # add threaded application classes
    appWrap = ApplicationWrapper()
    appWrap.start()

    try:
        ros_node()
    except rospy.ROSInterruptException:
        pass

    appWrap.join()


