#!/usr/bin/env python

from math import isnan, isinf
from numpy import interp
import rospy
import numpy as np

from vortex_msgs.msg import ThrusterForces, Pwm

THRUST_RANGE_LIMIT = 100

NUM_THRUSTERS = rospy.get_param('/propulsion/thrusters/num')
THRUST_OFFSET = rospy.get_param('/propulsion/thrusters/offset')
LOOKUP_THRUST = rospy.get_param('/propulsion/thrusters/characteristics/thrust')
LOOKUP_PULSE_WIDTH = rospy.get_param('/propulsion/thrusters/characteristics/pulse_width')
THRUSTER_DIRECTION = rospy.get_param('/propulsion/thrusters/direction')


def thrust_to_microsecs(thrust):
    return interp(thrust, LOOKUP_THRUST, LOOKUP_PULSE_WIDTH)


def healthy_message(msg):
    if len(msg.thrust) != NUM_THRUSTERS:
        rospy.logwarn_throttle(10, 'Wrong number of thrusters, ignoring...')
        return False

    for t in msg.thrust:
        if isnan(t) or isinf(t) or (abs(t) > THRUST_RANGE_LIMIT):
            rospy.logwarn_throttle(10, 'Message out of range, ignoring...')
            return False
    return True


class ThrusterInterface(object):
    def __init__(self):
        rospy.init_node('thruster_interface', anonymous=False)
        self.pub_pwm = rospy.Publisher('pwm', Pwm, queue_size=10)
        self.sub = rospy.Subscriber('/thrust/thruster_forces', ThrusterForces, self.callback)

        self.output_to_zero()
        rospy.on_shutdown(self.output_to_zero)
        rospy.loginfo('Initialized with thruster direction:\n\t{0}.'.format(THRUSTER_DIRECTION))
        rospy.loginfo('Initialized with offset:\n\t{0}.'.format(THRUST_OFFSET))

    def output_to_zero(self):
        zero_thrust_msg = ThrusterForces()
        for i in range(NUM_THRUSTERS):
            zero_thrust_msg.thrust.append(0)
        self.callback(zero_thrust_msg)

    def callback(self, msg):
        if not healthy_message(msg):
            return
        thrust = list(msg.thrust)

        microsecs = [None] * NUM_THRUSTERS
        pwm_msg = Pwm()

        for i in range(NUM_THRUSTERS):
            pwm_microsecs = thrust_to_microsecs(thrust[i]) + THRUST_OFFSET[i]
            
            if THRUSTER_DIRECTION[i] == -1:
                middle_value = 1500 + THRUST_OFFSET[i]
                diff = pwm_microsecs - middle_value
                pwm_microsecs = middle_value - diff 
            
            microsecs[i] = pwm_microsecs
            pwm_msg.pins.append(i)
            
        pwm_msg.positive_width_us = np.array(microsecs).astype('uint16')
        self.pub_pwm.publish(pwm_msg)


if __name__ == '__main__':
    try:
        thruster_interface = ThrusterInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
