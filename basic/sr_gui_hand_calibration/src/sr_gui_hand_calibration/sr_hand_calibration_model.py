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

import rospy

from sr_robot_lib.etherCAT_hand_lib import EtherCAT_Hand_Lib
from PyQt5.QtGui import QColor, QIcon
from PyQt5.QtWidgets import QTreeWidget, QTreeWidgetItem, QTreeWidgetItemIterator, QMessageBox
from PyQt5.QtCore import QTimer, pyqtSignal

import yaml
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper
import os

from collections import deque

green = QColor(153, 231, 96)
orange = QColor(247, 206, 134)
red = QColor(236, 178, 178)


class IndividualCalibration(QTreeWidgetItem):

    """
    Calibrate a single joint by raw and calibrated values
    Calibrated joints will appear as green
    or orange if calibrations are loaded from a file
    """

    def __init__(self, joint_name,
                 raw_value, calibrated_value,
                 parent_widget, tree_widget,
                 robot_lib):
        self.joint_name = joint_name
        self.raw_value = int(raw_value)
        self.calibrated_value = calibrated_value
        self.tree_widget = tree_widget
        self.robot_lib = robot_lib

        QTreeWidgetItem.__init__(self, parent_widget, [
                                 "", "", str(self.raw_value), str(self.calibrated_value)])

        for col in xrange(self.tree_widget.columnCount()):
            self.setBackground(col, QColor(red))

        self.tree_widget.addTopLevelItem(self)

        self.is_calibrated = False

    def remove(self):
        self.tree_widget.remove

    def calibrate(self):
        """
        Performs the joint calibration and sets background to green
        calibrate only the calibration lines, not the items for the fingers / joints / hand
        """
        self.raw_value = self.robot_lib.get_average_raw_value(
            self.joint_name, number_of_samples=100, accept_zeros=False)
        self.setText(2, str(self.raw_value))

        for col in xrange(self.tree_widget.columnCount()):
            if self.text(2) != "":
                self.setBackground(col, QColor(green))

        self.is_calibrated = True

    def set_is_loaded_calibration(self):
        """
        set the background to orange: those values are loaded
        from the file, not recalibrated
        """
        for col in xrange(self.tree_widget.columnCount()):
            if self.text(2) != "":
                self.setBackground(col, QColor(orange))

        self.is_calibrated = True

    def get_calibration(self):
        return [self.raw_value, self.calibrated_value]


class IndividualCalibrationCoupled(IndividualCalibration):

    """
    Calibrate coupled joints by raw and calibrated values
    Calibrated joints will appear as green
    or orange if calibrations are loaded from a file
    """

    def __init__(self, joint_names,
                 raw_values, calibrated_values,
                 parent_widget, tree_widget,
                 robot_lib):

        self.joint_names = joint_names
        self.raw_values = [int(raw_value) for raw_value in raw_values]
        self.calibrated_values = calibrated_values
        self.tree_widget = tree_widget
        self.robot_lib = robot_lib

        QTreeWidgetItem.__init__(self, parent_widget, [
                                 "", "", str(self.raw_values[0]) + ", " + str(self.raw_values[1]),
                                 str(self.calibrated_values[0]) + ", " + str(self.calibrated_values[1])])

        for col in xrange(self.tree_widget.columnCount()):
            self.setBackground(col, QColor(red))

        self.tree_widget.addTopLevelItem(self)

        self.is_calibrated = False

    def calibrate(self):
        """
        Performs the joint calibration and sets background to green
        calibrate only the calibration lines, not the items for the fingers / joints / hand
        """
        raw_values_str = []
        for idx in range(0, 2):
            self.raw_values[idx] = self.robot_lib.get_average_raw_value(
                self.joint_names[idx], 100)
            raw_values_str.append(str(self.raw_values[idx]))
        self.setText(2, ", ".join(raw_values_str))

        for col in xrange(self.tree_widget.columnCount()):
            if self.text(2) != "":
                self.setBackground(col, QColor(green))

        self.is_calibrated = True

    def get_calibration(self):
        return [self.raw_values, self.calibrated_values]


class JointCalibration(QTreeWidgetItem):

    """
    Calibrate a single joint by calibrations list
    Also displays the current joint position in the GUI
    """

    nb_values_to_check = 5

    def __init__(self, joint_name,
                 calibrations,
                 parent_widget, tree_widget,
                 robot_lib):
        self.joint_name = joint_name
        self.tree_widget = tree_widget
        self.robot_lib = robot_lib

        self.calibrations = []
        self.last_raw_values = deque()

        if type(self.joint_name) is not list:
            QTreeWidgetItem.__init__(
                self, parent_widget, ["", joint_name, "", ""])
            for calibration in calibrations:
                self.calibrations.append(IndividualCalibration(joint_name,
                                                               calibration[0], calibration[1],
                                                               self, tree_widget, robot_lib))
        else:
            QTreeWidgetItem.__init__(
                self, parent_widget, ["", joint_name[0] + ", " + joint_name[1], "", ""])
            for calibration in calibrations:
                self.calibrations.append(IndividualCalibrationCoupled(joint_name,
                                                                      calibration[0], calibration[1],
                                                                      self, tree_widget, robot_lib))

        # display the current joint position in the GUI
        self.timer = QTimer()
        self.timer.start(200)

        tree_widget.addTopLevelItem(self)
        self.timer.timeout.connect(self.update_joint_pos)

    def load_joint_calibration(self, new_calibrations):
        for calibration in self.calibrations:
            self.removeChild(calibration)
        self.calibrations = []

        for calibration in new_calibrations:
            if type(self.joint_name) is not list:
                new_calib = IndividualCalibration(self.joint_name,
                                                  calibration[0], calibration[1],
                                                  self, self.tree_widget, self.robot_lib)
            else:
                new_calib = IndividualCalibrationCoupled(self.joint_name,
                                                         calibration[0], [calibration[1], calibration[2]],
                                                         self, self.tree_widget, self.robot_lib)
            new_calib.set_is_loaded_calibration()
            self.calibrations.append(new_calib)

    def get_joint_calibration(self):
        config = []
        for calibration in self.calibrations:
            if calibration.is_calibrated:
                config.append(calibration.get_calibration())

        if len(config) <= 1:
            # no config, or only one point
            # generating flat config
            if type(self.joint_name) is not list:
                config = [[0, 0.0], [1, 0.0]]
            else:
                config = [[[0, 0], [0.0, 0.0]], [[1, 1], [0.0, 0.0]]]
        return [self.joint_name, config]

    def update_joint_pos(self):
        """
        Update the joint position if there are enough nonequal values
        If the values are equal it can be assumed the sensor are not measuring properly
        """
        if type(self.joint_name) is not list:
            raw_value = self.robot_lib.get_raw_value(self.joint_name)
            self.setText(2, str(raw_value))
        else:
            raw_value = []
            raw_value.append(self.robot_lib.get_raw_value(self.joint_name[0]))
            raw_value.append(self.robot_lib.get_raw_value(self.joint_name[1]))
            self.setText(2, str(raw_value_0) + ", " + str(raw_value_1))

        # if the 5 last values are equal, then display a warning
        # as there's always some noise on the values
        self.last_raw_values.append(raw_value)
        if len(self.last_raw_values) > self.nb_values_to_check:
            self.last_raw_values.popleft()

            # only check if we have enough values
            all_equal = True
            last_data = self.last_raw_values[0]
            for data in self.last_raw_values:
                if data != last_data:
                    all_equal = False
                    break
            if all_equal:
                self.setIcon(
                    0, QIcon(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../icons/warn.gif')))
                self.setToolTip(0, "No noise on the data for the last " + str(
                    self.nb_values_to_check) + " values, there could be a problem with the sensor.")
            else:
                self.setIcon(0, QIcon())
                self.setToolTip(0, "")

    def on_close(self):
        self.timer.stop()


class FingerCalibration(QTreeWidgetItem):

    """
    calibrate all joints of a finger
    """

    def __init__(self, finger_name,
                 finger_joints,
                 parent_widget, tree_widget,
                 robot_lib):

        QTreeWidgetItem.__init__(
            self, parent_widget, [finger_name, "", "", ""])

        self.joints = []
        for joint in finger_joints:
            self.joints.append(JointCalibration(joint_name=joint[0],
                                                calibrations=joint[1],
                                                parent_widget=self,
                                                tree_widget=tree_widget,
                                                robot_lib=robot_lib))

        tree_widget.addTopLevelItem(self)


class HandCalibration(QTreeWidgetItem):

    """
    calibrate all joints of all fingers of a hand
    """

    def __init__(self,
                 tree_widget,
                 progress_bar,
                 fingers=["First Finger", "Middle Finger",
                          "Ring Finger", "Little Finger",
                          "Thumb", "Wrist"],
                 old_version=False):

        self.old_version = old_version

        # TODO: Import this from an xml file?
        self.joint_map = {"First Finger": [["FFJ1", [[0.0, 0.0],
                                                     [0.0, 22.5],
                                                     [0.0, 45.0],
                                                     [0.0, 67.5],
                                                     [0.0, 90.0]]],
                                           ["FFJ2", [[0.0, 0.0],
                                                     [0.0, 22.5],
                                                     [0.0, 45.0],
                                                     [0.0, 67.5],
                                                     [0.0, 90.0]]],
                                           ["FFJ3", [[0.0, 0.0],
                                                     [0.0, 22.5],
                                                     [0.0, 45.0],
                                                     [0.0, 67.5],
                                                     [0.0, 90.0]]],
                                           ["FFJ4", [[0.0, -20.0],
                                                     [0.0, -10.0],
                                                     [0.0, 0.0],
                                                     [0.0, 10.0],
                                                     [0.0, 20.0]]]],

                          "Middle Finger": [["MFJ1", [[0.0, 0.0],
                                                      [0.0, 22.5],
                                                      [0.0, 45.0],
                                                      [0.0, 67.5],
                                                      [0.0, 90.0]]],
                                            ["MFJ2", [[0.0, 0.0],
                                                      [0.0, 22.5],
                                                      [0.0, 45.0],
                                                      [0.0, 67.5],
                                                      [0.0, 90.0]]],
                                            ["MFJ3", [[0.0, 0.0],
                                                      [0.0, 22.5],
                                                      [0.0, 45.0],
                                                      [0.0, 67.5],
                                                      [0.0, 90.0]]],
                                            ["MFJ4", [[0.0, -20.0],
                                                      [0.0, -10.0],
                                                      [0.0, 0.0],
                                                      [0.0, 10.0],
                                                      [0.0, 20.0]]]],

                          "Ring Finger": [["RFJ1", [[0.0, 0.0],
                                                    [0.0, 22.5],
                                                    [0.0, 45.0],
                                                    [0.0, 67.5],
                                                    [0.0, 90.0]]],
                                          ["RFJ2", [[0.0, 0.0],
                                                    [0.0, 22.5],
                                                    [0.0, 45.0],
                                                    [0.0, 67.5],
                                                    [0.0, 90.0]]],
                                          ["RFJ3", [[0.0, 0.0],
                                                    [0.0, 22.5],
                                                    [0.0, 45.0],
                                                    [0.0, 67.5],
                                                    [0.0, 90.0]]],
                                          ["RFJ4", [[0.0, -20.0],
                                                    [0.0, -10.0],
                                                    [0.0, 0.0],
                                                    [0.0, 10.0],
                                                    [0.0, 20.0]]]],

                          "Little Finger": [["LFJ1", [[0.0, 0.0],
                                                      [0.0, 22.5],
                                                      [0.0, 45.0],
                                                      [0.0, 67.5],
                                                      [0.0, 90.0]]],
                                            ["LFJ2", [[0.0, 0.0],
                                                      [0.0, 22.5],
                                                      [0.0, 45.0],
                                                      [0.0, 67.5],
                                                      [0.0, 90.0]]],
                                            ["LFJ3", [[0.0, 0.0],
                                                      [0.0, 22.5],
                                                      [0.0, 45.0],
                                                      [0.0, 67.5],
                                                      [0.0, 90.0]]],
                                            ["LFJ4", [[0.0, -20.0],
                                                      [0.0, -10.0],
                                                      [0.0, 0.0],
                                                      [0.0, 10.0],
                                                      [0.0, 20.0]]],
                                            ["LFJ5", [[0.0, 0.0],
                                                      [0.0, 22.5],
                                                      [0.0, 45.0],
                                                      [0.0, 67.5],
                                                      [0.0, 90.0]]]],

                          "Wrist": [["WRJ1", [[0.0, -45.0],
                                              [0.0, -22.5],
                                              [0.0, 0.0],
                                              [0.0, 15.0],
                                              [0.0, 30.0]]],
                                    ["WRJ2", [[0.0, -30.0],
                                              [0.0, 0.0],
                                              [0.0, 10.0]]]]
                          }

        if not self.old_version:
            self.joint_map["Thumb"] = [[["THJ1", "THJ2"], [[[0.0, 0.0], [0.0, 40]],
                                                           [[0.0, 0.0], [0.0, 20]],
                                                           [[0.0, 0.0], [0.0, 0.0]],
                                                           [[0.0, 0.0], [0.0, -20.0]],
                                                           [[0.0, 0.0], [0.0, -40.0]],
                                                           [[0.0, 0.0], [22.5, 40.0]],
                                                           [[0.0, 0.0], [22.5, 20.0]],
                                                           [[0.0, 0.0], [22.5, 0.0]],
                                                           [[0.0, 0.0], [22.5, -20.0]],
                                                           [[0.0, 0.0], [22.5, -40.0]],
                                                           [[0.0, 0.0], [45.0, 40.0]],
                                                           [[0.0, 0.0], [45.0, 20.0]],
                                                           [[0.0, 0.0], [45.0, 0.0]],
                                                           [[0.0, 0.0], [45.0, -20.0]],
                                                           [[0.0, 0.0], [45.0, -40.0]],
                                                           [[0.0, 0.0], [67.5, 40.0]],
                                                           [[0.0, 0.0], [67.5, 20.0]],
                                                           [[0.0, 0.0], [67.5, 0.0]],
                                                           [[0.0, 0.0], [67.5, -20.0]],
                                                           [[0.0, 0.0], [67.5, -40.0]],
                                                           [[0.0, 0.0], [90.0, 40.0]],
                                                           [[0.0, 0.0], [90.0, 20.0]],
                                                           [[0.0, 0.0], [90.0, 0.0]],
                                                           [[0.0, 0.0], [90.0, -20.0]],
                                                           [[0.0, 0.0], [90.0, -40]]]],
                                       ["THJ3", [[0.0, -15.0],
                                                 [0.0, 0.0],
                                                 [0.0, 15.0]]],
                                       ["THJ4", [[0.0, 0.0],
                                                 [0.0, 22.5],
                                                 [0.0, 45.0],
                                                 [0.0, 67.5]]],
                                       ["THJ5", [[0.0, -60.0],
                                                 [0.0, -30.0],
                                                 [0.0, 0.0],
                                                 [0.0, 30.0],
                                                 [0.0, 60.0]]]]
        else:
            self.joint_map["Thumb"] = [["THJ1", [[0.0, 0.0],
                                                 [0.0, 22.5],
                                                 [0.0, 45.0],
                                                 [0.0, 67.5],
                                                 [0.0, 90.0]]],
                                       ["THJ2", [[0.0, -40.0],
                                                 [0.0, -20.0],
                                                 [0.0, 0.0],
                                                 [0.0, 20.0],
                                                 [0.0, 40.0]]],
                                       ["THJ3", [[0.0, -15.0],
                                                 [0.0, 0.0],
                                                 [0.0, 15.0]]],
                                       ["THJ4", [[0.0, 0.0],
                                                 [0.0, 22.5],
                                                 [0.0, 45.0],
                                                 [0.0, 67.5]]],
                                       ["THJ5", [[0.0, -60.0],
                                                 [0.0, -30.0],
                                                 [0.0, 0.0],
                                                 [0.0, 30.0],
                                                 [0.0, 60.0]]]]

        self.fingers = []
        # this is set to False if the user doesn't want to continue
        # when there are no EtherCAT hand node currently running.
        self.is_active = True

        QTreeWidgetItem.__init__(self, ["Hand", "", "", ""])

        self.robot_lib = EtherCAT_Hand_Lib()
        if not self.robot_lib.activate():
            btn_pressed = QMessageBox.warning(
                tree_widget, "Warning", "The EtherCAT Hand node doesn't seem to be running, or the debug topic is not"
                " being published. Do you still want to continue? The calibration will be useless.",
                buttons=QMessageBox.Ok | QMessageBox.Cancel)

            if btn_pressed == QMessageBox.Cancel:
                self.is_active = False

        for finger in fingers:
            if finger in self.joint_map.keys():
                self.fingers.append(FingerCalibration(finger,
                                                      self.joint_map[finger],
                                                      self, tree_widget,
                                                      self.robot_lib))

            else:
                print finger, " not found in the calibration map"

        self.joint_0_calibration_index = 0

        self.progress_bar = progress_bar

        self.tree_widget = tree_widget
        self.tree_widget.addTopLevelItem(self)
        self.tree_widget.itemActivated.connect(self.calibrate_item)

    def unregister(self):
        it = QTreeWidgetItemIterator(self)
        while it.value():
            try:
                it.value().on_close()
            except:
                pass
            it += 1

        self.robot_lib.on_close()

    def calibrate_item(self, item):
        try:
            # only the IndividualCalibration have the calibrate method
            item.calibrate()
        except:
            pass

        self.progress()

        # select the next row by default
        item.setSelected(False)
        next_item = self.tree_widget.itemBelow(item)
        if next_item is not None:
            next_item.setSelected(True)
            self.tree_widget.setCurrentItem(next_item)

    def calibrate_joint0s(self, btn_joint_0s):
        joint0s = ["FFJ1", "FFJ2",
                   "MFJ1", "MFJ2",
                   "RFJ1", "RFJ2",
                   "LFJ1", "LFJ2"]

        it = QTreeWidgetItemIterator(self)
        while it.value():
            if it.value().text(1) in joint0s:
                it += self.joint_0_calibration_index + 1
                it.value().calibrate()
            it += 1

        self.joint_0_calibration_index += 1
        if self.joint_0_calibration_index == len(self.joint_map["First Finger"][0][1]):
            self.joint_0_calibration_index = 0

        # updating the btn text
        btn_joint_0s.setText(
            "Save all Joint 0s (angle = " +
            str(self.joint_map["First Finger"][0][1][self.joint_0_calibration_index][1]) + ")")

        self.progress()

    def progress(self):
        it = QTreeWidgetItemIterator(self)
        nb_of_items = 0
        nb_of_calibrated_items = 0
        while it.value():
            it += 1
            try:
                if it.value().is_calibrated:
                    nb_of_calibrated_items += 1
                nb_of_items += 1
            except:
                pass

        self.progress_bar.setValue(
            int(float(nb_of_calibrated_items) / float(nb_of_items) * 100.0))

    def load(self, filepath):
        f = open(filepath, 'r')
        document = ""
        for line in f.readlines():
            document += line
        f.close()
        yaml_config = yaml.load(document)

        if "sr_calibrations" not in yaml_config.keys():
            error_string = ('The selected calibration file does not contain calibration ' +
                            'values.')
            rospy.logwarn(error_string)
            QMessageBox(QMessageBox.Critical, 'Calibration file error', error_string).exec_()
            return

        if self.old_version:
            used_yaml_config = yaml_config["sr_calibrations"]
        else:
            if "sr_calibrations_coupled" not in yaml_config.keys():
                error_string = ('The selected calibration file does not contain coupled thumb calibration ' +
                                'values. Choose one that does, or switch to "Old Version" mode.')
                rospy.logwarn(error_string)
                QMessageBox(QMessageBox.Critical, 'Calibration file error', error_string).exec_()
                return
            used_yaml_config = yaml_config["sr_calibrations"] + yaml_config["sr_calibrations_coupled"]
        for joint in used_yaml_config:
            it = QTreeWidgetItemIterator(self)
            while it.value():
                if type(joint[0]) is not list:
                    joint_name = joint[0]
                else:
                    joint_name = ", ".join(joint[0])
                if it.value().text(1) == joint_name:
                    it.value().load_joint_calibration(joint[1])
                it += 1

        self.progress_bar.setValue(100)

    def save(self, filepath):
        yaml_config = {}

        joint_configs = []
        it = QTreeWidgetItemIterator(self)
        while it.value():
            try:
                joint_configs.append(it.value().get_joint_calibration())
            except:
                pass
            it += 1

        # this doesn't work as we'd like
        # yaml_config["sr_calibrations"] = joint_configs
        # full_config_to_write = yaml.dump(yaml_config,
        # default_flow_style=False)
        full_config_to_write = "sr_calibrations: [\n"
        for joint_config in joint_configs:
            if type(joint_config[0]) is not list:
                full_config_to_write += "[\""
                full_config_to_write += joint_config[0] + "\", "

                # the etherCAT driver wants floats
                for index, calib in enumerate(joint_config[1]):
                    joint_config[1][index][0] = float(calib[0])
                    joint_config[1][index][1] = float(calib[1])

                full_config_to_write += str(joint_config[1])
                full_config_to_write += "], \n"
        full_config_to_write += "]"

        if not self.old_version:
            full_config_to_write += "\n\nsr_calibrations_coupled: [\n"
            for joint_config in joint_configs:
                if type(joint_config[0]) is list:
                    full_config_to_write += "[[\""
                    full_config_to_write += joint_config[0][0] + "\", \"" + joint_config[0][1] + "\"], ["

                    for index, calib in enumerate(joint_config[1]):
                        joint_config[1][index][0][0] = float(calib[0][0])
                        joint_config[1][index][0][1] = float(calib[0][1])
                        joint_config[1][index][1][0] = float(calib[1][0])
                        joint_config[1][index][1][1] = float(calib[1][1])

                        if index > 0:
                            full_config_to_write += ", \n                    "
                        full_config_to_write += "["
                        full_config_to_write += str(joint_config[1][index][0]) + ", "
                        full_config_to_write += str(joint_config[1][index][1][0]) + ", " + \
                            str(joint_config[1][index][1][1])
                        full_config_to_write += "]"
                    full_config_to_write += "]]"
            full_config_to_write += "\n]"

        f = open(filepath, 'w')
        f.write(full_config_to_write)
        f.close()

    def is_calibration_complete(self):
        it = QTreeWidgetItemIterator(self)
        while it.value():
            try:
                if not it.value().is_calibrated:
                    return False
            except:
                pass
            it += 1

        return True
