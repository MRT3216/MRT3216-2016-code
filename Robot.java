package org.usfirst.frc.team3216.robot;
// all them imports:
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

import com.ni.vision.NIVision;

public class Robot extends IterativeRobot {
	// setting up all the objects
	// global objects:
	Joystick xBox; // xbox controller, obvoiusly (well actually a logitech gamepad)
	
	Compressor pcm; // pneumatics compressor and solenoid controller
	PowerDistributionPanel pdp; // to get voltage/amperage stuff
	DriverStation ds; // getting DS state, other info
	NetworkTable table; // sending data back & forth for sensors
	
	VictorSP leftdrive,rightdrive; // y-cable these outputs to the two speed controllers (2 motors per side)
	TimedDoubleSolenoid shift; // connect the two shifter pneumatic actuators in parallel
	Victor ballmotor; // motor to push/pull ball in and out
	TimedDoubleSolenoid ballholder; // two parallel pneumatic actuators to lift ball and hold it in place
	
	AnalogInput range, leftpressure, rightpressure; // range = analog out of ultrasonic rangefinder, *pressure = analog pressure resistors on the front
	Timer arduinoTimer, gearTimer, ballTimer, liftTimer; // timers for handling timed events
	AnalogInput balllimit; // limit switch to tell if the ball has been taken in all the way
	Servo lballservo,rballservo; // two servos that close to hold the ball in place
	
	REVDigitBoard disp; // digit board connected to MXP 
	//SerialPort arduino; // this never worked
	Arduino arduino; // wrapper class to make I2C setup easier
	MovingAverage rangefinder; // smooth spikes in the rangefinder input by averaging the last several samples
	
	final int numcameras = 1; // we've got this many cameras
	int[] cameras; // array of camera indices
	int curcamera; // current camera index 
	NIVision.Image curframe; // current frame to stream to camera
	CameraServer cserver; // camera server instance
	
	static boolean autonstart = false, autonend = false; // to help handle the 3-stage auton: startup, periodic, end

	/* Connections:
	 * 
	 * left motors: victor on pwm 0
	 * right motors: victor on pwm 1
	 * gearshift: solenoid 0&1
	 * 
	 * rangefinder: analog input 0
	 * pressure sensors on front: analog 2&3
	 * 
	 * ball holder: solenoid 2&3
	 * ball motor: victor on pwm 2
	 * left ball holder: servo on pwm 3
	 * right ball holder: servo on pwm 4
	 * ball limit switch: analog input 1
	 * 
	 * rev digit board on MXP
	 * onboard i2c to an arduino
	 */
	
	/* Controls:
	 * 
	 * vertical joystick axes: tank drive left and right
	 * right trigger: intake a ball, both triggers vary the speed of the motors
	 * left trigger: eject a ball
	 * left bumper: switch to low gear (only while held)
	 */
	
	public void robotInit() {
		/// prefs: here we instantiate values and stuff
		Settings.add("pressure", 10,0,1024);// setpoint for pressure sensors to trigger the automatic low gear switch
		Settings.add("autondist", 250,0,1024); // distance to move in autonomous (cm iirc)
		Settings.add("deadzone", 0.07,0,1); // deadzone in joysticks
		Settings.add("triggerdz", 0.07,0,1); // trigger deadzone
		Settings.add("solenoidtimer", 0.3,0,1); // timer for gearshift delay (in seconds)
		Settings.add("arduinotimer", 0.2,0,3); // send data to arduino every x seconds
		Settings.add("balltimer", 0.3,0,3); // seconds to spin up motors before ejecting ball
		Settings.add("balllimit", 1000,0,4000); // analog value for when to trigger the ball holding machanism 
		Settings.add("servoopen", 130,0,180); // degrees to open the servos when not holding the ball
		Settings.add("servoclose", 50,0,180); // degrees to close servos when holding the ball
		Settings.add("autonspeed", 0.5, 0, 1); // speed to drive in auton
		Settings.add("motormap", 0.7, 0, 1); // motor slow down factor
		Settings.add("ballmotormap", 0.7, 0, 1);
		Settings.add("lifttimer", 0.6, 0.1, 4);
		
		/// now we set up the obects
		xBox = new Joystick(0); // joystick port 0
		pcm = new Compressor(0); // compressor on can network at id 0
		table = NetworkTable.getTable("datatable"); // this table communicates back to the computer for diagnostic purposes
		pdp = new PowerDistributionPanel(); // pdp objecto to read amperages, etc.
		ds = DriverStation.getInstance(); // to get match info for LEDs
		
		leftdrive = new VictorSP(0); // left motors = pwm 0
		rightdrive = new VictorSP(1); // right motors = pwm 1
		shift = new TimedDoubleSolenoid(1,0); // shifter pneumtics on solenoid 0,1
		
		ballholder = new TimedDoubleSolenoid(3,2); // ball holder pneumatics on solenoid 2,3
		ballmotor = new Victor(2); // ball suck/eject motor = pwm 2
		lballservo = new Servo(3); // left-side servo to hold ball
		rballservo = new Servo(4); // right-side servo to hold ball
		balllimit = new AnalogInput(3); // mechanical or light-based limit switch
		
		range = new AnalogInput(0); // analog rangefinder
		rangefinder = new MovingAverage(5,250); // moving average for rangefinder (samples, start value)
		
		leftpressure = new AnalogInput(1); // force-sensitive resistors
		rightpressure = new AnalogInput(2);
		
		disp = new REVDigitBoard(); // REV digit board object
		disp.clear(); // clear any prevoius data
		disp.display("load"); // indicate that the robot is loading. this will be overwritten in the sendData periodic function
		
		/*try {
			arduino = new SerialPort(19200,SerialPort.Port.kUSB); //  this never did work, used I2C instead
		} catch (RuntimeException a) { }*/
		arduino = new Arduino((byte)84); // arduino i2c slave is at 84
		
		gearTimer = new Timer(); // timer to hold gear solenoids open long enough to switch, but we don't want to waste air
		arduinoTimer = new Timer(); // timer to send data to the arduino at intervals to no flood the buffers
		arduinoTimer.start(); // start it so we can send data
		ballTimer = new Timer(); // timer for ball pneumatics (same reason as gear)
		liftTimer = new Timer();
		
		pcm.setClosedLoopControl(true); // this is now done programatically by the pneumatics module
		
		// set up the camera multiplex system 
		try {
		curframe = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0); // blank image (placeholder)
		for (int i = 0; i < numcameras; i++) 
			cameras[i] = NIVision.IMAQdxOpenCamera("cam" + (i+1), NIVision.IMAQdxCameraControlMode.CameraControlModeController); // get the camera ID
		curcamera = 0;
		NIVision.IMAQdxConfigureGrab(curcamera); // turn it on
		cserver = CameraServer.getInstance(); // get the instance
		} catch (Exception e) { }
		
		autonstart = false; // make sure these are false for the third time 
		autonend = false;
	}

	// This function is called periodically during autonomous
	public void autonomousPeriodic() {
		sendData(); // this also does the moving average stuff
		
		if (!autonstart) {
			ballholder.backward(); // close the ball holder
			closeServos();
			manageGears(0,0,false); // set high gear
			autonstart = true;
		} else if (autonstart && !autonend) {
			double speed = 0.5; //map(a_range, 0, 200, 0, 1)*0.5+0.1; // this was a variable speed formula but it didn't work
			if (rangefinder.getAverage() > Settings.get("autondist")) { // if more than setpoint distance
				drive(speed,speed);
			} else {
				autonend = true;
			}
		} else if (autonend) {
			drive(0,0);
		}
	}
	
	public void teleopInit() {
		ballholder.backward();
		closeServos();
	}

	// variables used in teleop:
	double leftstick, rightstick, triggerr, triggerl, pressurer, pressurel; // these are data points pulled off the xbox controller
	boolean buttonl, buttonr, limit; // also xbox controller stuff
	boolean lastbuttonr; // for debouncing the camera toggle
	// This function is called periodically during operator control
	public void teleopPeriodic() {
		// this took a lot of guess-and-check, since several 
		// axes were messed up this year. don't trust the 
		// joystick explorer, use the DS for testing these
		leftstick = xBox.getRawAxis(1); // these are supposed to be the vertical axes (for tank drive)
		rightstick = xBox.getRawAxis(5); // checked
		
		buttonl = xBox.getRawButton(6); // left bumper overrides the gearshift and forces it into low gear (high by default)
		buttonr = xBox.getRawButton(5); 
 
		triggerl = xBox.getRawAxis(2); // these spin up the motor to inject/eject the boulder
		triggerr = xBox.getRawAxis(3);	
		
		pressurer = rightpressure.getVoltage(); // get the voltage from the force-sensitive resistors
		pressurel = leftpressure.getVoltage();
		
		limit = balllimit.getValue() > Settings.get("balllimit"); // inreases when the ball gets closer; setpoint is 0-1024
		//limit = false;
		
		if (buttonr) {
			ballholder.backward();
			closeServos();
		}
		
		drive(leftstick, -rightstick); // drive function
		
		manageGears(pressurel, pressurer, buttonl); // gearshift logic code
		
		manageBall(triggerl-triggerr, limit); // ball logic and timing code
		
		/*if (buttonr && !lastbuttonr) switchCamera(); // right bumper switches cameras
		lastbuttonr = buttonr; // store this so we don't continuously switch while the button is held
		*/
		
		sendData(); // periodic function
	}
	
	/* the following useful functions:
	 * 
	 * drive(double left_joystick, double right_joystick) : moves the motors, handles deadzone and ideally reversing stuff
	 * manageGears(double analog_pressure_left, double analog_pressure_right, boolean override_switch) : automatic gearshifting with manual override
	 * manageBall(double direction, boolean limit) : moves the ball in or out (probably just a motor controller)
	 */
	
	//////////////////////////////// various management functions
	
	void drive(double left, double right) {
		if (Math.abs(left) > Settings.get("deadzone")) { // deadzone the motors
			leftdrive.set(Math.pow(left,3)*Settings.get("motormap")); // cubic motor map
		} else {
			leftdrive.set(0);
		}
		
		if (Math.abs(right) > Settings.get("deadzone")) {
			rightdrive.set(Math.pow(right,3)*Settings.get("motormap"));
		} else {
			rightdrive.set(0);
		}
	}
	
	boolean timerRunning = false; // for some reason, timers don't have a field to figure this out
	boolean intake = false;
	boolean eject = false;
	boolean close = false;
	
	void manageBall(double direction, boolean limit) {
		// positive values eject, negative values intake
		// ideally ballholder.forward will open it
		// this is really messy logic
		if (direction < -Settings.get("deadzone")) { // if the ball is coming in
			if (!timerRunning) {
				ballholder.forward(); // drop the plate
				openServos(); // open servos to accept the ball
			}
			ballmotor.set(direction*Settings.get("ballmotormap")); // spin up the motor in reverse to intake
			intake = true; // set these values 
			eject = false;
			close = false;
		} else if (direction > Settings.get("deadzone")) {
			ballmotor.set(direction*Settings.get("ballmotormap")); // spin up the motor to eject
			if (!timerRunning) { // to time the ball holder: it needs to spin up the motors before the holder drops the ball for maximum speed ejection
				ballTimer.start();
				timerRunning = true;
			}
			eject = true; // set these
			intake = false;
			close = false;
		} else {
			ballmotor.set(0); // stop the motor, and stop the timers (TODO: fix)
			if (!intake) {
				eject = false;
				timerRunning = false;
				ballTimer.stop();
				ballTimer.reset();
			}
			//intake = false;
			if (!close) {
				liftTimer.reset();
				liftTimer.start();
				close = true;
			}
		}
		//// for the eject mode, we want to spin up the motors and wait a sec to eject the ball
		if (timerRunning && eject && ballTimer.get() > Settings.get("balltimer")) { // if waiting to eject, and time has passed
			openServos(); // open holder servos
			ballholder.forward(); // drop the plate
			ballTimer.stop(); // stop and reset timer
			ballTimer.reset();
			timerRunning = false;
			eject = false; 
		}
		//// for intake mode, we need to wait until the limit is triggered and then lift the plate, and then wait a sec and close the servos
		if (timerRunning && intake && ballTimer.get() > Settings.get("balltimer")) {
			closeServos(); // close the servos after the timer has expired
			ballTimer.stop(); // and reset the variables
			//timerRunning = false;
			//intake = false;
		} else if (intake && limit) { // if limit switch is tripped and intaking
			if (!timerRunning) { 
				ballTimer.reset();
				ballTimer.start(); // if timer hasn't started, start it
				timerRunning = true;
				ballholder.backward(); // and lift the plate as soon as the limit is tripped
			}
		}
		if (close && liftTimer.get() > Settings.get("lifttimer")) { // to automatically lift plate so it doesn't break off
			ballholder.backward();
			//closeServos();
			liftTimer.stop(); // stop timer
			close = false;
		}
	}

	boolean lastgear = true; // true = high, false = low gear
	
	void manageGears(double lpressure, double rpressure, boolean override) {
		// not sure what to do with the encoder/speed thing yet (double lencoder, double rencoder, double lspeed, double rspeed,) <-- add that in
		boolean gear = !(lpressure > Settings.get("pressure") || rpressure > Settings.get("pressure") || override);

		if (gear != lastgear && gear) {
			shift.forward();
		} else if (gear != lastgear && !gear) {
			shift.backward();
		}
		lastgear = gear;
	}
	
	void openServos() {
		lballservo.setAngle(Settings.get("servoopen"));
		rballservo.setAngle(180-Settings.get("servoopen"));
	}
	
	void closeServos() {
		lballservo.setAngle(Settings.get("servoclose"));
		rballservo.setAngle(180-Settings.get("servoclose"));
		
	}

	////////////////////////// miscellaneous stuffs
	
	public void testPeriodic() {
		sendData(); // send data in test
	}
	
	public void disabledPeriodic() {
		sendData(); // send data in disabled
	}
	
	double map(double value, double istart, double istop, double ostart, double ostop){ // to map stuff from one range to another
		return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
	}
	
	// this kinda became the all-encompassing function to handle periodic tasks.
	void sendData() {
		//moving average for rangefinder
		double a_range = range.getValue();; // centimeters hopefully
		rangefinder.newSample(a_range);
		
		TimedDoubleSolenoid.run(); // run all the solenoids
		
		Settings.sync(); // this syncs local sttings with the NetworkTable and the DS config utility
		
		syncSensors();
		
		//digit board
		try {
			disp.display(ControllerPower.getInputVoltage()); //live voltage readout ideally, or whatever we need
		} catch (RuntimeException a) { } // to handle I2C errors?
		
		// now do light stuff
		runLights();
		/*
		// manage camera 
		NIVision.IMAQdxGrab(cameras[curcamera], curframe, 1); // copy the image
		cserver.setImage(curframe); // I hope this doesn't lag anything*/
	}
	
	void runLights() {
		try {
			if (arduinoTimer.get() > Settings.get("arduinotimer")) { // send every 200msec to avoid buffer overflows
				byte mode1 = 0;  //////// structure: 0b<red><blue><fms><auton><teleop><disabled><enabled><attached>
				if (ds.getAlliance() == DriverStation.Alliance.Red)  mode1 |= 0b10000000; // on the red alliance
				if (ds.getAlliance() == DriverStation.Alliance.Blue) mode1 |= 0b01000000; // blue alliance
				if (ds.isFMSAttached())                              mode1 |= 0b00100000; // FMS (on the field) or just at home on a computer
				if (ds.isAutonomous())                               mode1 |= 0b00010000; // auton mode
				if (ds.isOperatorControl())                          mode1 |= 0b00001000; // teleop mode
				if (ds.isDisabled())                                 mode1 |= 0b00000100; // disabled (idk what this actually means)
				if (ds.isEnabled())                                  mode1 |= 0b00000010; // enabled and running
				if (ds.isDSAttached())                               mode1 |= 0b00000001; // might not even work if it's not connected
				
				//byte[] mode2 = {mode1}; arduino.write(mode2, 1);
				arduino.sendbyte(mode1); // send the byte of status over
				
				arduinoTimer.reset(); // reset the timer so we can 
				arduinoTimer.start();  //probably don't need to do this
			}
		} catch (RuntimeException a) { } // catch I2C errors
	}
	
	void switchCamera() {
		NIVision.IMAQdxStopAcquisition(cameras[curcamera]); // stop current camera (might actually not want to start/stop so often
		curcamera += 1; curcamera %= numcameras; // advance counter
		NIVision.IMAQdxConfigureGrab(cameras[curcamera]); // start new camera (disabled)
	}
	
	void syncSensors() {
		try { // put data into table (probably disable this during comp
			table.putNumber("pwr_v",pdp.getVoltage()); // PDP voltage (not the same as DS voltage)
			table.putNumber("pwr_t",pdp.getTemperature()); // useful to tell if there are things heating up
			table.putNumber("pwr_c",pdp.getTotalCurrent()); // total current draw
			for (int i = 0; i < 16; i++) table.putNumber("pwr_c_" + i,pdp.getCurrent(i)); // current draw for all 16 channels
			table.putNumber("pcm_c",pcm.getCompressorCurrent()); // compressor current draw
			table.putNumber("range",rangefinder.getAverage()); // averaged rangefinder value
			table.putNumber("ctl_v",ControllerPower.getInputVoltage()); // roborio voltage
			table.putNumber("ctl_c",ControllerPower.getInputCurrent()); // roborio current draw
			table.putNumber("ctl_fault",ControllerPower.getFaultCount3V3()+ControllerPower.getFaultCount5V()+ControllerPower.getFaultCount6V()); // total voltage fault count
			table.putNumber("inf_range",balllimit.getValue());
		} catch (RuntimeException a) { } // runtime exception could be caused by CAN timeout
	}
}
