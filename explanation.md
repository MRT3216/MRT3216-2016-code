Some explanation of all this code is probably nessecary...

The main ideas of this code are explained here:

Autonomous:
- lift the ball holder plate (we're going to be holding a ball hopefully)
- close the ball holder servos
- Loop:
	- drive forward until the ultrasonic rangefinder falls below a certain value
- and then stop once it has reached that value

Teleop:
- read the values from the controller
- tank drive from joystick values
- gearshift logic:
	- if the pressure sensors on the front pass a certain setpoint, or the override button is held, set the low gear flag
	- if the flag is different from last time, trigger a timed solenoid operation
- ball holder logic:
	- for intaking:
		- drop the plate and open the servos
		- spin up the motors to pull in
		- once the IR rangefinder registers the ball at a certain distance, lift the plate
		- wait a time interval and then close the servos when the plate is lifted and in place
	- for ejecting:
		- lift the plate and close the servos in preparation
		- spin up the motors to push out
		- wait a time interval in order to let them get up to speed
		- then open the servos and drop the plate so the ball hits the high speed motors and is ejected
- if the camera switch button is pressed, switch the camera view

Periodic (always runs):
- get rangefinder value and send it to the MovingAverage class
- run all the timed solenoid operations
- synchronize the preferences like setpoints from internal memory and network tables
- send local sensor data to the network tables
- display battery voltage on the digit board
- push driverstation info the the LED strips
- push camera frames to the camera server
