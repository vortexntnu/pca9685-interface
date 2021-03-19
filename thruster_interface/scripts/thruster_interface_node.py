#!/usr/bin/env python

from math import isnan, isinf
from numpy import interp
import rospy
import numpy as np

from vortex_msgs.msg import ThrusterForces, Pwm


class ThrusterInterface(object):
    def __init__(self):

        self.pwm_pub = None
        self.sub = None

        self.output_to_zero()
        rospy.on_shutdown(self.output_to_zero)
        rospy.loginfo('Initialized with thruster direction:\n\t{0}.'.format(
            THRUSTER_DIRECTION))
        rospy.loginfo('Initialized with offset:\n\t{0}.'.format(THRUST_OFFSET))

    def thrust_to_microsecs(self, thrust):
        return interp(thrust, LOOKUP_THRUST, LOOKUP_PULSE_WIDTH)

    def healthy_message(self, msg):
        if len(msg.thrust) != NUM_THRUSTERS:
            rospy.logwarn_throttle(10,
                                   'Wrong number of thrusters, ignoring...')
            return False

        for t in msg.thrust:
            if isnan(t) or isinf(t) or (abs(t) > THRUST_RANGE_LIMIT):
                rospy.logwarn_throttle(10, 'Message out of range, ignoring...')
                return False
        return True

    def thrust_cb(self, msg):
        if not self.healthy_message(msg):
            return
        thrust = list(msg.thrust)

        microsecs = [None] * NUM_THRUSTERS
        pwm_msg = Pwm()

        for i in range(NUM_THRUSTERS):
            pwm_microsecs = self.thrust_to_microsecs(
                thrust[i]) + THRUST_OFFSET[i]

            if THRUSTER_DIRECTION[i] == -1:
                middle_value = 1500 + THRUST_OFFSET[i]
                diff = pwm_microsecs - middle_value
                pwm_microsecs = middle_value - diff

            microsecs[i] = pwm_microsecs
            pwm_msg.pins.append(i)

        pwm_msg.positive_width_us = np.array(microsecs).astype('uint16')
        self.pwm_pub.publish(pwm_msg)

    def output_to_zero(self):
        zero_thrust_msg = ThrusterForces()
        for i in range(NUM_THRUSTERS):
            zero_thrust_msg.thrust.append(0)
        self.thrust_cb(zero_thrust_msg)


if __name__ == '__main__':
    rospy.init_node('thruster_interface')

    THRUST_RANGE_LIMIT = 100
    NUM_THRUSTERS = rospy.get_param('/propulsion/thrusters/num')
    THRUST_OFFSET = rospy.get_param('/propulsion/thrusters/offset')
    LOOKUP_THRUST = rospy.get_param(
        '/propulsion/thrusters/characteristics/thrust')
    LOOKUP_PULSE_WIDTH = rospy.get_param(
        '/propulsion/thrusters/characteristics/pulse_width')
    THRUSTER_DIRECTION = rospy.get_param('/propulsion/thrusters/direction')

    thruster_interface = ThrusterInterface()

    thruster_interface.pwm_pub = rospy.Publisher('pwm', Pwm, queue_size=10)
    thruster_interface.thrust_sub = rospy.Subscriber(
        '/thrust/thruster_forces', ThrusterForces,
        thruster_interface.thrust_cb)
    
    rospy.spin()
