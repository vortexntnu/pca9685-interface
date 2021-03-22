#!/usr/bin/env python

from math import isnan, isinf
import rospy
import numpy as np

from std_msgs.msg import Int32, Float32
from vortex_msgs.msg import Pwm


class ThrusterInterface(object):
    def __init__(self, path_to_mapping, voltage_topic, thurster_forces_topic,
                 pwm_topic, thrust_forward_limit, thrust_backward_limit,
                 num_thrusters, thruster_directions, thruster_offsets):

        self.num_thrusters = num_thrusters
        self.thruster_directions = thruster_directions
        self.thruter_offsets = thruster_offsets
        self.thrust_forward_limit = thrust_forward_limit
        self.thrust_backward_limit = thrust_backward_limit

        self.voltage = None
        self.voltage_sub = rospy.Subscriber(voltage_topic, Int32,
                                            self.voltage_cb)
        rospy.loginfo('waiting for voltage on {voltage_topic}..')
        rospy.wait_for_message(voltage_topic, Int32)  # voltage must be set
        self.pwm_pub = rospy.Publisher(pwm_topic, Pwm, queue_size=10)
        self.thrust_sub = rospy.Subscriber(thurster_forces_topic,
                                           Float32, self.thrust_cb)

        self.output_to_zero()
        rospy.on_shutdown(self.output_to_zero)

        rospy.loginfo('Thruster interface initialized')

    def zero_thrust_msg(self):
        """creates a ThrusterForces message with all thrusts set to zero

        Returns:
            ThrusterForces: message with all thrusts set to zero
        """
        zero_thrust_msg = Float32()
        for i in range(NUM_THRUSTERS):
            zero_thrust_msg.data.append(0)
        return zero_thrust_msg

    def thrust_to_microsecs(self, thrust):
        return np.interp(thrust, LOOKUP_THRUST, LOOKUP_PULSE_WIDTH)

    def validate_and_limit_thrust(self, thrust_msg):
        """limits a thrust value to achieveable maximum and minumum values and
        performs sanity checks on the message. Returns zero thrust is desired 
        thrust is insane and returns a limited thurst if desired thrusts are 
        out of bounds.

        Args:
            thrust_msg (Float32): msg with desired thrusts in newton

        Returns:
            Float32: msg with achievable thrusts in newton
        """
        if len(thrust_msg.data) != NUM_THRUSTERS:
            rospy.logerr('Wrong number of thrusters, setting thrust to zero')
            return self.zero_thrust_msg()

        for thruster_number in range(thrust_msg.data):
            thrust = thrust_msg.data[thruster_number]
            if isnan(thrust) or isinf(thrust):
                rospy.logerr(
                    'Desired thrust Nan or Inf, setting thrust to zero')
                return self.zero_thrust_msg()
            if thrust > self.thrust_forward_limit:
                rospy.logerr(
                    'Thruster {thruster_number} limited to maximum forward thrust'
                )
                thrust_msg.data[thruster_number] = self.thrust_forward_limit
            if thrust < self.thrust_backward_limit:
                rospy.logerr(
                    'Thruster {thruster_number} limited to maximum backward thrust'
                )
                thrust_msg.data[thruster_number] = self.thrust_backward_limit

        return thrust_msg

    def thrust_cb(self, thrust_msg):
        """Takes inn desired thruster forces and publishes corresponding desired pwm values.

        Args:
            thrust_msg (Float32): desired thruster forces in newton
        """
        thrust_msg = self.validate_and_limit_thrust(thrust_msg)
        thrust = list(thrust_msg.data)

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
        """Sets thrust to zero
        """
        zero_thrust_msg = self.zero_thrust_msg()
        self.thrust_cb(zero_thrust_msg)

    def voltage_cb(self, voltage_msg):
        self.voltage = voltage_msg.data


if __name__ == '__main__':
    rospy.init_node('thruster_interface', log_level=rospy.INFO)

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

    thruster_interface = ThrusterInterface(path_to_thruster_mapping,
                                           voltage_topic, thrust_topic,
                                           pwm_topic)

    rospy.spin()
