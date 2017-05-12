#!/usr/bin/env python
import sys
import rospy
import unittest
import rostest
from StringIO import StringIO
from mock import patch

from sr_gui_cyberglove_calibrator.cyberglove_calibrer import CybergloveCalibrer

NAME = "test_cyberglove_calibrator"
PKG = "sr_gui_cyberglove_calibrator"


class TestCybergloveCalibrator(unittest.TestCase):

    def setUp(self):
        self.filename = "mock_name"
        self.held, sys.stdout = sys.stdout, StringIO()

    @patch('sr_gui_cyberglove_calibrator.cyberglove_calibrer.Cyberglove')
    def test_warning_message(self, Cyberglove):
        self.calibrer = CybergloveCalibrer(description_function=None)
        self.calibrer.load_calib(self.filename)
        self.assertEqual(sys.stdout.getvalue().strip(), 'Call start service not found, is the driver running? ' +
                                                        'If you are using cyberglove_trajectory, ' +
                                                        'please be adviced that following plugin does ' +
                                                        'not support that package yet.')

if __name__ == "__main__":
    rospy.init_node("test_cyberglove_calibrator")
    rostest.rosrun(PKG, NAME, TestCybergloveCalibrator)
