#!/usr/bin/env python

from math import isnan, isinf
import rospy
import numpy as np

from std_msgs.msg import Int32
from vortex_msgs.msg import ThrusterForces, Pwm


class ThrusterInterface(object):
    def __init__(self, path_to_mapping, voltage_topic, thurster_forces_topic,
                 pwm_topic):

        self.voltage = None

        self.voltage_sub = rospy.Subscriber(voltage_topic, Int32,
                                            self.voltage_cb)
        rospy.wait_for_message(voltage_topic, Int32)  # voltage must be set
        self.pwm_pub = rospy.Publisher(pwm_topic, Pwm, queue_size=10)
        self.thrust_sub = rospy.Subscriber(thurster_forces_topic,
                                           ThrusterForces, self.thrust_cb)

        self.output_to_zero()
        rospy.on_shutdown(self.output_to_zero)

        rospy.loginfo('Thruster interface initialized')

    def thrust_to_microsecs(self, thrust):
        return np.interp(thrust, LOOKUP_THRUST, LOOKUP_PULSE_WIDTH)

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
            pwm_microsecs = self.thrust_to_microsecs(thrust[i]) + THRUST_OFFSET[i]

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

    def voltage_cb(self, voltage_msg):
        self.voltage = voltage_msg.data


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

    pwm_topic = 'pwm'
    voltage_topic = '/auv/battery_level'
    thrust_topic = '/thrust/thruster_forces'
    
    path_to_thruster_mapping = ''

    thruster_interface = ThrusterInterface(
        path_to_thruster_mapping,
        voltage_topic,
        thrust_topic,
        pwm_topic
    )

    rospy.spin()
