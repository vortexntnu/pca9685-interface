#!/usr/bin/python3

from math import isnan, isinf
from collections import deque
import numpy as np
from scipy.interpolate import interp1d
from openpyxl import load_workbook

import rospy, rospkg
from std_msgs.msg import Float32, Float32MultiArray
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
        self.thruster_offsets = thruster_offsets

        self.thruster_operational_voltage_range = rospy.get_param(
            "/propulsion/thrusters/thrusters_operational_voltage_range"
        )
        self.thruster_operational_voltage_min = self.thruster_operational_voltage_range[
            0
        ]
        self.thruster_operational_voltage_max = self.thruster_operational_voltage_range[
            1
        ]

        self.thruster_map = rospy.get_param("/propulsion/thrusters/map")
        self.thrust_range = rospy.get_param("/propulsion/thrusters/thrust_range")
        self.pwm_limit_min = self.map_percentage_to_pwm(
            self.thrust_range[0], 1100, 1900
        )
        self.pwm_limit_max = self.map_percentage_to_pwm(
            self.thrust_range[1], 1100, 1900
        )

        # create thruster to pwm lookup function
        rospy.loginfo("Parsing and interpolating thruster datasheet..")
        (
            self.pwm_values,
            self.thrusts_from_voltage,
            self.thrust_to_pwm,
        ) = self.parse_and_interpolate_thruster_data(thruster_datasheet_path)

        # set up subscribers and publishers
        self.voltage_queue = deque([10] * 10)
        self.voltage_sub = rospy.Subscriber(voltage_topic, Float32, self.voltage_cb)
        # rospy.loginfo("waiting for voltage on %s.." % voltage_topic)
        # rospy.wait_for_message(voltage_topic, Float32)  # voltage must be set
        self.pwm_pub = rospy.Publisher(pwm_topic, Pwm, queue_size=1)
        self.thrust_sub = rospy.Subscriber(
            thurster_forces_topic, Float32MultiArray, self.thrust_cb
        )
        self.delivered_thrust_pub = rospy.Publisher(
            "/thrust/delivered_forces", Float32MultiArray, queue_size=1
        )

        self.output_to_zero()
        rospy.on_shutdown(self.output_to_zero)

        rospy.loginfo("Thruster interface initialized")

    def map_percentage_to_pwm(self, percentage, rangeStart, rangeEnd):
        """maps percentage values from -1.9 to 1.0 into start and range pwm signal

        Args:
            percentage: Percentage that will be maped [-1.0, 1.0]
            rangeStart: PWM sart range
            rangeEnd: PWM end range

        Returns:
            list: all possible pwm values
            dict[float]: dictionary with thrust for each pwm for a given voltage
            dict[float]: dictionary with thrust_to_pwm interpolation functions
                            for a given voltage
        """

        pwmSignal = int(((percentage + 1) / 2) * (rangeEnd - rangeStart) + rangeStart)
        return pwmSignal

    def parse_and_interpolate_thruster_data(self, thruster_datasheet_path):
        """Parses the blue robotics T200 datasheet and creates a new dataset
        for voltages at steps of 0.1 by interpolating existing data.

        Args:
            thruster_datasheet_path (string): path to T200 thruster dataseheet

        Returns:
            list: all possible pwm values
            dict[float]: dictionary with thrust for each pwm for a given voltage
            dict[float]: dictionary with thrust_to_pwm interpolation functions
                            for a given voltage
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
            for voltage, thrust in zip(new_voltage_steps, interpolated_thrusts):
                thrusts_from_voltage[voltage].append(thrust)

        # create pwm_lookup functions by 1d interpolation of thrusts at each voltage level
        thrust_to_pwm = dict()
        for voltage in new_voltage_steps:
            thrust_to_pwm[voltage] = interp1d(
                thrusts_from_voltage[voltage], pwm_values, kind="slinear"
            )

        return (pwm_values, thrusts_from_voltage, thrust_to_pwm)

    def pwm_lookup(self, thrust, voltage):
        """finds a good pwm value for a desired thrust and a battery voltage

        Args:
            thrust (Float32): desired thruster forces
            voltage (Float): battery voltage

        Returns:
            Int: pwm signal
        """
        if (
            thrust == 0
        ):  # neccessary since multiple pwms map to zero thrust, which confuses the interpolation
            return 1500
        else:
            voltage_rounded = np.round(voltage, decimals=1)
            pwm = self.thrust_to_pwm[voltage_rounded](thrust)
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

    def get_voltage(self):
        return np.round(sum(self.voltage_queue) / len(self.voltage_queue), 1)

    def add_voltage(self, voltage):
        self.voltage_queue.popleft()
        self.voltage_queue.append(voltage)

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
        thruster_forces = list(thrust_msg.data)
        voltage = self.get_voltage()

        if len(thrust_msg.data) != self.num_thrusters:
            return self.zero_thrust_msg()

        for thruster_number in range(len(thrust_msg.data)):
            forward_limit = self.thrusts_from_voltage[voltage][
                -self.thruster_offsets[thruster_number] // 4 - 5
            ]  # 4 because of steps provided in T200 datasheet, 5 because of a quick hack
            reverse_limit = self.thrusts_from_voltage[voltage][
                self.thruster_offsets[thruster_number] // 4 + 5
            ]
            thrust = thrust_msg.data[thruster_number]

            if isnan(thrust) or isinf(thrust):
                return self.zero_thrust_msg()
            if thrust > forward_limit:
                thruster_forces[thruster_number] = forward_limit
            if thrust < reverse_limit:
                thruster_forces[thruster_number] = reverse_limit

        return Float32MultiArray(data=thruster_forces)

    def limit_pwm(self, pwm, i):
        """limits pwm to a preset amount, has a hardcoded limiter for min/max values just in case
        Args:
            pwm (Int): a pwm value
            i: thruster nr.?

        Returns:
            double: int limited pwm value
        """

        # Softcoded limit that can be changed
        if pwm > (self.pwm_limit_max + self.thruster_offsets[self.thruster_map[i]]):
            pwm = self.pwm_limit_max + self.thruster_offsets[self.thruster_map[i]]
        elif pwm < (self.pwm_limit_min + self.thruster_offsets[self.thruster_map[i]]):
            pwm = self.pwm_limit_min + self.thruster_offsets[self.thruster_map[i]]

        # A hardcoded limit if someone sets thrust ofset to high, we limit it to 1900 or 1100 PWM
        if pwm > 1900:
            return 1900
        elif pwm < 1100:
            return 1100

        return pwm

    def output_to_zero(self):
        """Sets thrust to zero"""
        zero_thrust_msg = self.zero_thrust_msg()
        self.thrust_cb(zero_thrust_msg)

    def voltage_cb(self, voltage_msg):
        """Rounds voltage to one decimal and saves it

        Args:
            voltage_msg (Float32): voltage in volt
        """
        self.add_voltage(voltage_msg.data)

    def thrust_from_pwm(self, pwm):
        """returns thrust for a given pwm, taking system voltage into account.

        Args:
            pwm (Int): a pwm value

        Returns:
            double: thrust value
        """
        return self.pwm_to_thrust[self.get_voltage()](pwm)

    def thrust_cb(self, thrust_msg):
        """Takes inn desired thruster forces and publishes
        corresponding desired pwm values.

        Args:
            thrust_msg (Float32): desired thruster forces in newton
        """

        # check that voltage is within range
        voltage = self.get_voltage()
        if not (
            self.thruster_operational_voltage_min
            <= voltage
            <= self.thruster_operational_voltage_max
        ):
            self.STOP_thrusters()
            return

        # validate thrust
        validated_thrust_msg = self.validate_and_limit_thrust(thrust_msg)
        thrust = list(validated_thrust_msg.data)

        # calculate pwm values from desired thrust and system voltage
        microsecs = [None] * self.num_thrusters
        pwm_msg = Pwm()
        for i in range(self.num_thrusters):
            pwm_microsecs = (
                self.pwm_lookup(thrust[self.thruster_map[i]], voltage)
                + self.thruster_offsets[i]
            )

            if self.thruster_directions[self.thruster_map[i]] == -1:
                middle_value = 1500 + self.thruster_offsets[self.thruster_map[i]]
                diff = pwm_microsecs - middle_value
                pwm_microsecs = middle_value - diff
            pwm_microsecs = self.limit_pwm(pwm_microsecs, i)
            microsecs[self.thruster_map[i]] = pwm_microsecs
            pwm_msg.pins.append(self.thruster_map[i])

        # publish pwm
        pwm_msg.positive_width_us = np.array(microsecs).astype("uint16")
        self.pwm_pub.publish(pwm_msg)

        # publish delivered thrust (for data collection purposes)
        self.delivered_thrust_pub.publish(validated_thrust_msg)

    def STOP_thrusters(self):
        # calculate pwm values to STOP thusters
        microsecs = [None] * self.num_thrusters
        pwm_msg = Pwm()
        for i in range(self.num_thrusters):
            middle_value = 1500 + self.thruster_offsets[self.thruster_map[i]]
            microsecs[self.thruster_map[i]] = middle_value  # Thrust to 0%
            pwm_msg.pins.append(self.thruster_map[i])

        # publish pwm
        pwm_msg.positive_width_us = np.array(microsecs).astype("uint16")
        self.pwm_pub.publish(pwm_msg)

        # publish delivered thrust (for data collection purposes)
        self.delivered_thrust_pub.publish(self.zero_thrust_msg())


if __name__ == "__main__":
    rospy.init_node("thruster_interface", log_level=rospy.INFO)
    rospack = rospkg.RosPack()
    thruster_interface_path = rospack.get_path("thruster_interface")

    PWM_TOPIC = rospy.get_param("/thruster_interface/pwm_topic", default="/pwm")
    VOLTAGE_TOPIC = rospy.get_param(
        "/thruster_interface/voltage_topic", default="/auv/battery_level/system"
    )
    DESIRED_THRUST_TOPIC = rospy.get_param(
        "/thruster_interface/desired_thrust_topic", default="/thrust/thruster_forces"
    )

    T200_DATASHEET_PATH = rospy.get_param(
        "/thruster_interface/thruster_datasheet_path",
        default="%s/config/T200-Public-Performance-Data-10-20V-September-2019.xlsx"
        % thruster_interface_path,
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
