#!/usr/bin/env python

import rospy
from vortex_msgs.msg import Pwm
import Adafruit_PCA9685

# Constants
PWM_BITS_PER_PERIOD = rospy.get_param('/pca9685/pwm/bits_per_period')
FREQUENCY = rospy.get_param('/pca9685/pwm/frequency')
FREQUENCY_MEASURED = rospy.get_param('/pca9685/pwm/frequency_measured')
PERIOD_LENGTH_IN_MICROSECONDS = 1000000.0 / FREQUENCY_MEASURED
PWM_ON = 0 # Start of duty cycle


class Pca9685InterfaceNode(object):
    def __init__(self):
        rospy.init_node('pwm_node')
        self.sub = rospy.Subscriber('pwm', Pwm, self.callback, queue_size=1)

        addr = rospy.get_param('/i2c/address')
        bus = rospy.get_param('/i2c/bus')
        try:
            self.pca9685 = Adafruit_PCA9685.PCA9685(address=addr, busnum=bus)
            self.pca9685.set_pwm_freq(FREQUENCY)
            self.pca9685.set_all_pwm(0, 0)
            self.current_pwm = [0]*16
        except Exception as e:
            rospy.logerr(e)
            


        rospy.on_shutdown(self.shutdown)

        rospy.loginfo('Initialized for {0} Hz.'.format(FREQUENCY))

    def callback(self, msg):
        if len(msg.pins) == len(msg.positive_width_us):
            for i in range(len(msg.pins)):
                if msg.positive_width_us[i] != self.current_pwm[msg.pins[i]]:
                    self.pca9685.set_pwm(msg.pins[i], PWM_ON, self.microsecs_to_bits(msg.positive_width_us[i]))
                    self.current_pwm[msg.pins[i]] = msg.positive_width_us[i]
        print(msg.positive_width_us) #troubleshooting
        print(msg.pins)              #troubleshooting

    def microsecs_to_bits(self, microsecs):
        duty_cycle_normalized = microsecs / PERIOD_LENGTH_IN_MICROSECONDS
        return int(round(PWM_BITS_PER_PERIOD * duty_cycle_normalized))

    def remap(self, old_val, old_min, old_max, new_min, new_max):
        # Remaps an interval. Used for testing purposes.
        return (new_max - new_min)*(old_val - old_min) / (old_max - old_min) + new_min

    def shutdown(self):
        self.pca9685.set_all_pwm(0, 0)

if __name__ == '__main__':
    try:
        pwm_node = Pca9685InterfaceNode()
        rospy.spin()
    except IOError:
        rospy.logerr('IOError caught, shutting down.')
    except rospy.ROSInterruptException:
        pass