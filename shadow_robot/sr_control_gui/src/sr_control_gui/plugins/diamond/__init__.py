#!/usr/bin/env python
#
# Copyright 2011 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#


import roslib; roslib.load_manifest('sr_control_gui')
import rospy

from PyQt4 import QtCore, QtGui, Qt

import threading, math, time
from functools import partial
from config import Config
from generic_plugin import GenericPlugin
from std_msgs.msg import Float64


class BaseMovement(object):
    def __init__(self, finger_name):
        self.finger_name = finger_name

        self.msg_to_send_j3 = Float64()
        self.msg_to_send_j3.data = 0.0

        self.msg_to_send_j4 = Float64()
        self.msg_to_send_j4.data = 0.0

        self.sleep_time = 0.0001

        topic = "/sh_"+ finger_name.lower() +"j3_position_controller/command"
        self.publisher_j3 = rospy.Publisher(topic, Float64)
        topic = "/sh_"+ finger_name.lower() +"j4_position_controller/command"
        self.publisher_j4 = rospy.Publisher(topic, Float64)


    def publish(self, mvt_percentage):
        self.update(mvt_percentage)
        self.publisher_j4.publish(self.msg_to_send_j3)
        self.publisher_j3.publish(self.msg_to_send_j4)
        time.sleep(self.sleep_time)

    def update(self, mvt_percentage):
        pass

    def close(self):
        self.publisher_j3.unregister()
        self.publisher_j4.unregister()
        self.publisher_j3 = None
        self.publisher_j4 = None

class EllipsoidMovement(BaseMovement):
    def __init__(self, finger_name, amplitude_j3 = 0.15, amplitude_j4 = 0.3):
        BaseMovement.__init__(self,finger_name)
        self.amplitude_j3 = amplitude_j3
        self.amplitude_j4 = amplitude_j4

    def update(self, mvt_percentage):
        j3 = self.amplitude_j3 * math.sin(2.0*3.14159 * mvt_percentage/100.) 
        j4 = 2.0*self.amplitude_j4 * math.cos(2.0*3.14159 * mvt_percentage/100.) 
        self.msg_to_send_j3.data = j3
        self.msg_to_send_j4.data = j4


class Movement(threading.Thread):
    def __init__(self, finger_name):
        threading.Thread.__init__(self)
        self.moving = False
        self.finger_name = finger_name
        self.iterations = 10000
        self.movements = []
        
        for i in range(1, 5):
            self.movements.append( EllipsoidMovement(finger_name, amplitude_j3=(float(i)/10.0)*0.05, amplitude_j4=(float(i)/10.0)*0.05) )
        for i in range(0, 4):
            self.movements.append( EllipsoidMovement(finger_name, amplitude_j3=((5-float(i))/10.0)*0.05, amplitude_j4=((5.-float(i))/10.0)*0.05) )

        
    def run(self):
        while(True):
            for movement in self.movements:
                for mvt_percentage in range(0, self.iterations):
                    if self.moving == False:
                        return
                    else:
                        movement.publish(mvt_percentage/(self.iterations/100.))

    def close(self):
        self.moving = False
        for movement in movements:
            movement.close()

class Diamond(GenericPlugin):
    """
    Follows a diamond with FJ3 and FJ4
    """
    name = "Diamond"

    def __init__(self):
        GenericPlugin.__init__(self)

        self.fingers = ["FF","MF","RF","LF"]
        self.layout = QtGui.QHBoxLayout()
        self.frame = QtGui.QFrame()
        self.buttons = {}
        self.movements = {}
        self.moving = {}
        for finger in self.fingers:
            self.movements[finger] = Movement(finger)
            self.moving[finger] = False
            tmp_btn = QtGui.QPushButton(finger)
            self.buttons[finger] = tmp_btn
            tmp_btn.clicked.connect(partial(self.clicked, finger))

            self.layout.addWidget( tmp_btn )
        self.frame.setLayout( self.layout )
        self.window.setWidget( self.frame )

        self.dependencies = None

    def clicked(self, finger_name):
        if self.moving[finger_name]:
            self.movements[finger_name].moving = False
            self.moving[finger_name] = False
            self.movements[finger_name].join()
            self.movements[finger_name] = None
        else:
            self.moving[finger_name] = True
            self.movements[finger_name] = Movement(finger_name)
            self.movements[finger_name].moving = True
            self.movements[finger_name].start()

    def activate(self):
        GenericPlugin.activate(self)

    def on_close(self):
        for movement in self.movements.values():
            movement.close()

        OpenGLGenericPlugin.on_close(self)
