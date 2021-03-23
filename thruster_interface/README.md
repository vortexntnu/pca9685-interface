# thruster_interface

Mapps desired thruster forces to corresponding pwm signals while taking battery voltage into consideration. 

Subscribes to:
* /thrust/thruster_forces

Publihses:
* /pwm

Params (default value in brackets):
* __thruster offsets__ /propulsion/thrusters/offset
* __thruster directions__ /propulsion/thrusters/direction
* __number of thrusters__ /propulsion/thrusters/num (8)
* __pwm topic__ /thruster_interface/pwm_topic (/pwm)
* __battery voltage topic__ /thruster_interface/voltage_topic (/auv/battery_level)
* __desired thrust topic__ /thruster_interface/desired_thrust_topic (/thrust/thruster_forces)
* __thruster datasheet path__ /thruster_interface/thruster_datasheet_path ($(find thruster_interface)/config/T200-Public-Performance-Data-10-20V-September-2019.xlsx)

## Dependencies
* The [Adafruit Python PCA9685](https://github.com/adafruit/Adafruit_Python_PCA9685) driver
`sudo pip install adafruit-pca9685`
* The [NumPy](http://www.numpy.org/) Python library
`sudo apt install python-numpy`
* Working I2C

