#!/usr/bin/env python

from math import isnan, isinf
import rospy
import numpy as np
from openpyxl import load_workbook

from std_msgs.msg import Int32, Float32MultiArray
from vortex_msgs.msg import Pwm


class ThrusterInterface(object):
    def __init__(
        self,
        thruster_datasheet_path,
        voltage_topic,
        thurster_forces_topic,
        pwm_topic,
        num_thrusters,
        thruster_directions,
        thruster_offsets,
    ):

        self.num_thrusters = num_thrusters
        self.thruster_directions = thruster_directions
        self.thruter_offsets = thruster_offsets

        # create thruster to pwm lookup function
        rospy.loginfo("Parsing and interpolating thruster datasheet..")
        (
            self.pwm_values,
            self.thrusts_from_voltage,
        ) = self.parse_and_interpolate_thruster_data(thruster_datasheet_path)

        # set up subscribers and publishers
        self.voltage = None
        self.voltage_sub = rospy.Subscriber(voltage_topic, Int32, self.voltage_cb)
        rospy.loginfo("waiting for voltage on {voltage_topic}..")
        rospy.wait_for_message(voltage_topic, Int32)  # voltage must be set
        self.pwm_pub = rospy.Publisher(pwm_topic, Pwm, queue_size=10)
        self.thrust_sub = rospy.Subscriber(
            thurster_forces_topic, Float32MultiArray, self.thrust_cb
        )

        self.output_to_zero()
        rospy.on_shutdown(self.output_to_zero)

        rospy.loginfo("Thruster interface initialized")

    def parse_and_interpolate_thruster_data(self, thruster_datasheet_path):
        """Parses the blue robotics T200 datasheet and creates a new dataset
        for voltages at steps of 0.1 by interpolating existing data.

        Args:
            thruster_datasheet_path (string): path to T200 thruster dataseheet

        Returns:
            Tuple[list, dict]: array with pwm values and
                               dictionary with thrust for each pwm for a given voltage
        """

        # parse T200 datasheet
        workbook = load_workbook(filename=thruster_datasheet_path)
        voltages = [10, 12, 14, 16, 18, 20]
        pwm_values = [cell[0].value for cell in workbook["10 V"]["A2":"A202"]]
        thrusts_dict = dict()
        for voltage in voltages:
            thrusts_dict[voltage] = [
                cell[0].value * 9.8 for cell in workbook["%i V" % voltage]["F2":"F202"]
            ]  # * 9.8 for converting from kg f to newton

        # create new dataset with voltage steps of 0.1 by interpolating
        new_voltage_steps = np.round(np.arange(10, 20.01, 0.1), decimals=1)
        thrusts_from_voltage = dict()
        for voltage in new_voltage_steps:
            thrusts_from_voltage[voltage] = []
        for i in range(len(pwm_values)):
            pwm = pwm_values[i]
            thrusts = [thrusts_dict[voltage][i] for voltage in voltages]
            interpolated_thrusts = np.interp(new_voltage_steps, voltages, thrusts)

            # save interpolated thrusts to dict
            for (voltage, thrust) in zip(new_voltage_steps, interpolated_thrusts):
                thrusts_from_voltage[voltage].append(thrust)

        return (pwm_values, thrusts_from_voltage)

    def pwm_lookup(self, thrust, voltage):
        """finds a good pwm value for a desired thrust and a battery voltage

        Args:
            thrust (Float32): desired thruster forces
            voltage (Int32): battery voltage

        Returns:
            Int: pwm signal 
        """
        voltage_rounded = np.round(voltage, decimals=1)
        pwm = np.interp(
            thrust, self.thrusts_from_voltage[voltage_rounded], self.pwm_values
        )
        return int(np.round(pwm))

    def zero_thrust_msg(self):
        """creates a ThrusterForces message with all thrusts set to zero

        Returns:
            ThrusterForces: message with all thrusts set to zero
        """
        zero_thrust_msg = Float32MultiArray()
        for i in range(self.num_thrusters):
            zero_thrust_msg.data.append(0)
        return zero_thrust_msg

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
        thrust_forward_limit = self.thrusts_from_voltage[self.voltage][-1]
        thrust_reverse_limit = self.thrusts_from_voltage[self.voltage][0]

        thruster_forces = list(thrust_msg.data)

        if len(thrust_msg.data) != self.num_thrusters:
            rospy.logerr("Wrong number of thrusters, setting thrust to zero")
            return self.zero_thrust_msg()

        for thruster_number in range(len(thrust_msg.data)):
            thrust = thrust_msg.data[thruster_number]
            if isnan(thrust) or isinf(thrust):
                rospy.logerr("Desired thrust Nan or Inf, setting thrust to zero")
                return self.zero_thrust_msg()
            if thrust > thrust_forward_limit:
                rospy.logerr(
                    "Thruster %i limited to maximum forward thrust %f N" % (thruster_number, thrust_forward_limit)
                )
                thruster_forces[thruster_number] = thrust_forward_limit
            if thrust < thrust_reverse_limit:
                rospy.logerr(
                    "Thruster %i limited to maximum reverse thrust %f N" % (thruster_number, thrust_reverse_limit)
                )
                thruster_forces[thruster_number] = thrust_reverse_limit

        return Float32MultiArray(data=thruster_forces)

    def limit_pwm(self, pwm, thruster_number):
        if pwm > 1900:
            rospy.logerr(
                'Too high desired forward thrust pwm on thruster %i. Limited to upper limit 1900' % thruster_number
                )
            return 1900
        if pwm < 1100:
            rospy.logerr(
                'Too high desired reverse thrust pwm on thruster %i. Limited to lower limit 1100' % thruster_number
                )
            return 1100
        return pwm

    def thrust_cb(self, thrust_msg):
        """Takes inn desired thruster forces and publishes
        corresponding desired pwm values.

        Args:
            thrust_msg (Float32): desired thruster forces in newton
        """
        thrust_msg = self.validate_and_limit_thrust(thrust_msg)
        thrust = list(thrust_msg.data)

        microsecs = [None] * self.num_thrusters
        pwm_msg = Pwm()

        for i in range(self.num_thrusters):
            pwm_microsecs = self.pwm_lookup(thrust[i], self.voltage) + self.thruter_offsets[i]

            if self.thruster_directions[i] == -1:
                middle_value = 1500 + self.thruter_offsets[i]
                diff = pwm_microsecs - middle_value
                pwm_microsecs = middle_value - diff

            pwm_microsecs = self.limit_pwm(pwm_microsecs, i)

            microsecs[i] = pwm_microsecs
            pwm_msg.pins.append(i)

        pwm_msg.positive_width_us = np.array(microsecs).astype("uint16")
        self.pwm_pub.publish(pwm_msg)

    def output_to_zero(self):
        """Sets thrust to zero"""
        zero_thrust_msg = self.zero_thrust_msg()
        self.thrust_cb(zero_thrust_msg)

    def voltage_cb(self, voltage_msg):
        """Maps voltage from millivolt to volts and saves it

        Args:
            voltage_msg (Int32): voltage in millivolt
        """
        self.voltage = np.round(voltage_msg.data / 1000, 1)


if __name__ == "__main__":
    rospy.init_node("thruster_interface", log_level=rospy.INFO)

    PWM_TOPIC = rospy.get_param("/thruster_interface/pwm_topic", default="/pwm")
    VOLTAGE_TOPIC = rospy.get_param(
        "/thruster_interface/voltage_topic", default="/auv/battery_level"
    )
    DESIRED_THRUST_TOPIC = rospy.get_param(
        "/thruster_interface/desired_thrust_topic", default="/thrust/thruster_forces"
    )

    T200_DATASHEET_PATH = rospy.get_param(
        "/thruster_interface/thruster_datasheet_path",
        default='config/T200-Public-Performance-Data-10-20V-September-2019.xlsx',
    )
    NUM_THRUSTERS = rospy.get_param("/propulsion/thrusters/num", default=8)
    THRUST_OFFSET = rospy.get_param("/propulsion/thrusters/offset")
    THRUSTER_DIRECTION = rospy.get_param("/propulsion/thrusters/direction")

    thruster_interface = ThrusterInterface(
        thruster_datasheet_path=T200_DATASHEET_PATH,
        voltage_topic=VOLTAGE_TOPIC,
        thurster_forces_topic=DESIRED_THRUST_TOPIC,
        pwm_topic=PWM_TOPIC,
        num_thrusters=NUM_THRUSTERS,
        thruster_directions=THRUSTER_DIRECTION,
        thruster_offsets=THRUST_OFFSET,
    )

    rospy.spin()
