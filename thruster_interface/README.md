# thruster_interface

Subscribes to:
* /thrust/thruster_forces

Publihses:
 * /pwm

## Dependencies
* The [Adafruit Python PCA9685](https://github.com/adafruit/Adafruit_Python_PCA9685) driver
`sudo pip install adafruit-pca9685`
* The [NumPy](http://www.numpy.org/) Python library
`sudo apt install python-numpy`
* Working I2C

## Notes
To run the node without a PCA9685 connected, set `thrusters_connected = false` in the launch script. To connect the PWM board to the Raspberry Pi (or other host computer) connect VCC to a 3.3 V pin, SCL to SCL, SDA to SDA, and ground to ground.
