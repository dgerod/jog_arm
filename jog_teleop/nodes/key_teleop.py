#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013 PAL Robotics SL.
# Released under the BSD License.
#
# Authors:
#   * Siegfried-A. Gevatter

import curses
import math
import rospy
from geometry_msgs.msg import Twist

from geometry_msgs.msg import TwistStamped
from jog_msgs.msg import JogJoint


class Velocity(object):

    def __init__(self, min_velocity, max_velocity, num_steps):

        assert min_velocity > 0 and max_velocity > 0 and num_steps > 0
        self._min = min_velocity
        self._max = max_velocity
        self._num_steps = num_steps
        if self._num_steps > 1:
            self._step_incr = (max_velocity - min_velocity) / (self._num_steps - 1)
        else:
            # If num_steps is one, we always use the minimum velocity.
            self._step_incr = 0

    def __call__(self, value, step):
        """
        Takes a value in the range [0, 1] and the step and returns the
        velocity (usually m/s or rad/s).
        """

        if step == 0:
            return 0

        assert step > 0 and step <= self._num_steps
        max_value = self._min + self._step_incr * (step - 1)
        return value * max_value


class TextWindow:

    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):

        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):

        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError('lineno out of bounds')
        height, width = self._screen.getmaxyx()
        y = int((height / self._num_lines) * lineno)
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(y, x, text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()

class KeyTeleop:

    TWIST_CMD_TOPIC_NAME = 'jog_arm_server/cartesian_command'
    JOG_CMD_TOPIC_NAME = 'jog_arm_server/joint_command'

    def __init__(self, interface):

        self._hz = rospy.get_param('~hz', 10)
        self._num_steps = rospy.get_param('~turbo/steps', 4)

        forward_min = rospy.get_param('~turbo/linear_forward_min', 0.5)
        forward_max = rospy.get_param('~turbo/linear_forward_max', 1.0)
        self._calculate_forward_velocity = Velocity(forward_min, forward_max, self._num_steps)

        backward_min = rospy.get_param('~turbo/linear_backward_min', 0.25)
        backward_max = rospy.get_param('~turbo/linear_backward_max', 0.5)
        self._calculate_backward_velocity = Velocity(backward_min, backward_max, self._num_steps)

        angular_min = rospy.get_param('~turbo/angular_min', 0.7)
        angular_max = rospy.get_param('~turbo/angular_max', 1.2)
        self._calculate_rotation_velocity = Velocity(angular_min, angular_max, self._num_steps)

        self._running = True
        self._linear = 0
        self._angular = 0

        self._interface = interface
        self._twist_pub, self._joint_pub = self._init_comms()

    def run(self):

        rate = rospy.Rate(self._hz)
        while self._running:
            keycode = self._interface.read_key()
            if keycode:
                if self._key_pressed(keycode):
                    self._publish()
            else:
                self._publish()
                rate.sleep()

    def _init_comms(self):

        twist_cmd_pub = rospy.Publisher(self.TWIST_CMD_TOPIC_NAME, TwistStamped)
        joint_cmd_pub = rospy.Publisher(self.JOG_CMD_TOPIC_NAME, JogJoint)
        return twist_cmd_pub, joint_cmd_pub

    def _prepare_twist_command(self, linear, angular):

        ts = TwistStamped()
        if linear >= 0:
            ts.twist.linear.z = self._calculate_forward_velocity(1.0, linear)
        else:
            ts.twist.linear.z = self._calculate_backward_velocity(-1.0, -linear)

        ts.twist.angular.z = self._calculate_rotation_velocity(math.copysign(1, angular), abs(angular))
        return ts

    def _prepare_joint_command(self, linear, angular):

        jj = JogJoint()
        return jj

    def _key_pressed(self, keycode):

        movement_bindings = {
            curses.KEY_UP: (1, 0),
            curses.KEY_DOWN: (-1, 0),
            curses.KEY_LEFT: (0, 1),
            curses.KEY_RIGHT: (0, -1),
        }
        speed_bindings = {
            ord(' '): (0, 0),
        }

        if keycode in movement_bindings:
            acc = movement_bindings[keycode]
            ok = False
            if acc[0]:
                linear = self._linear + acc[0]
                if abs(linear) <= self._num_steps:
                    self._linear = linear
                    ok = True
            if acc[1]:
                angular = self._angular + acc[1]
                if abs(angular) <= self._num_steps:
                    self._angular = angular
                    ok = True
            if not ok:
                self._interface.beep()

        elif keycode in speed_bindings:
            acc = speed_bindings[keycode]
            if acc[0] is not None:
                self._linear = acc[0]
            if acc[1] is not None:
                self._angular = acc[1]

        elif keycode == ord('q'):
            self._running = False
            rospy.signal_shutdown('Bye')
        else:
            return False

        return True

    def _publish(self):

        self._interface.clear()
        self._interface.write_line(2, 'Linear: %d, Angular: %d' % (self._linear, self._angular))
        self._interface.write_line(5, 'Use arrow keys to move, space to stop, q to exit.')
        self._interface.refresh()

        twist_cmd = self._prepare_twist_command(self._linear, self._angular)
        joint_cmd = self._prepare_joint_command(self._linear, self._angular)
        self._twist_pub.publish(twist_cmd)
        self._joint_pub.publish(joint_cmd)


def main(stdscr):
    rospy.init_node('key_teleop')
    app = KeyTeleop(TextWindow(stdscr))
    app.run()


if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
