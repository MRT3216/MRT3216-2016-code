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
	
	VictorSP leftdrive,rightdrive; // y-cable thiede outputs to the two speed controllers (2 motors per side)
	TimedDoubleSolenoid shift; // connect the two shifter pneumatic actuators in parallel
	Victor ballmotor; // motor to push/pull ball in and out
	TimedDoubleSolenoid ballholder; // two parallel pneumatic actuators to lift ball and hold it in place
	
	AnalogInput range, leftpressure, rightpressure; // range = analog out of ultrasonic rangefinder, *pressure = analog pressure resistors on the front
	Timer arduinoTimer, gearTimer, ballTimer; // timers for handling timed events
	DigitalInput balllimit; // limit switch to tell if the ball has been taken in all the way
	Servo lballservo,rballservo; // two servos that close to hold the ball in place
	
	REVDigitBoard disp; // digit board connected to MXP
	//SerialPort arduino; // this never worked
	Arduino arduino; // wrapper class to make I2C setup easier
	MovingAverage rangefinder; // smooth spikes in the rangefinder input by averaging the last several samples
	
	final int numcameras = 2; // we've got this many cameras
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
	 * ball motor: voctor on pwm 2
	 * left ball holder: servo on pwm 3
	 * right ball holder: servo on pwm 4
	 * ball limit switch: digital input 0
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
		Settings.add("pressuresp", 10);// setpoint for pressure sensors to trigger the automatic low gear switch
		Settings.add("autondistsp", 250); // distance to move in autonomous (cm iirc)
		Settings.add("deadzonesp", 0.15); // deadzone in joysticks
		Settings.add("triggerdzsp", 0.15); // trigger deadzone
		Settings.add("solenoidtimersp", 0.3); // timer for gearshift delay (in seconds)
		Settings.add("arduinotimersp", 0.2); // send data to arduino every x seconds
		Settings.add("balltimersp", 0.3); // seconds to spin up motors before ejecting ball
		Settings.add("servoopen", 130); // degrees to open the servos when not holding the ball
		Settings.add("servoclose", 70); // degrees to close servos when holding the ball
		
		/// now we set up the obects
		xBox = new Joystick(0); // joystick port 0
		pcm = new Compressor(0); // compressor on can network at id 0
		table = NetworkTable.getTable("datatable"); // this table communicates back to the computer for diagnostic purposes
		pdp = new PowerDistributionPanel(); // pdp objecto to read amperages, etc.
		ds = DriverStation.getInstance(); // to get match info for LEDs
		
		leftdrive = new VictorSP(0); // left motors = pwm 0
		rightdrive = new VictorSP(1); // right motors = pwm 1
		shift = new TimedDoubleSolenoid(0,1); // shifter pneumtics on solenoid 0,1
		
		ballholder = new TimedDoubleSolenoid(2,3); // ball holder pneumatics on solenoid 2,3
		ballmotor = new Victor(2); // ball suck/eject motor = pwm 2
		lballservo = new Servo(3); // left-side servo to hold ball
		rballservo = new Servo(4); // right-side servo to hold ball
		balllimit = new DigitalInput(0); // mechanical or light-based limit switch
		
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
		
		pcm.setClosedLoopControl(true); // this is now done programatically by the pneumatics module
		
		// set up the camera multiplex system (DISABLED)
		/*  curframe = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0); // blank image (placeholder)
		for (int i = 0; i < numcameras; i++) {
			cameras[i] = NIVision.IMAQdxOpenCamera("cam" + (i+1), NIVision.IMAQdxCameraControlMode.CameraControlModeController); // get the camera ID
			NIVision.IMAQdxConfigureGrab(cameras[i]); // turn it on
		}
		curcamera = 0;
		cserver = CameraServer.getInstance(); // get the instance */
		
		autonstart = false; // make sure these are false for the third time 
		autonend = false;
	}

	// This function is called periodically during autonomous
	public void autonomousPeriodic() {
		sendData(); // this also does the moving average stuff
		
		if (!autonstart) {
			// TODO: set up the stuff
			ballholder.forward(); // open the ball holder
			openServos();
			autonstart = true;
		} else if (autonstart && !autonend) {
			double speed = 0.5; //map(a_range, 0, 200, 0, 1)*0.5+0.1; // this was a variable speed formula but it didn't work
			if (rangefinder.getAverage() < Settings.get("autondistsp")) {
				drive(speed,speed);
			} else {
				autonend = true;
			}
		} else if (autonend) {
			drive(0,0);
		}
	}

	// variables used in teleop:
	double leftstick, rightstick, triggerr, triggerl, pressurer, pressurel; // these are data points pulled off the xbox controller
	boolean buttonl, buttonr, limit; // also xbox controller stuff
	// This function is called periodically during operator control
	public void teleopPeriodic() {
		// this took a lot of guess-and-check, since several 
		// axes were messed up this year. don't trust the 
		// joystick explorer, use the DS for testing these
		leftstick = xBox.getRawAxis(1); // these are supposed to be the vertical axes (for tank drive)
		rightstick = xBox.getRawAxis(3);
		
		buttonl = xBox.getRawButton(6); // left bumper overrides the gearshift and forces it into low gear (high by default)
		buttonr = xBox.getRawButton(5); 
 
		triggerl = xBox.getRawAxis(2); // these spin up the motor to inject/eject the boulder
		triggerr = xBox.getRawAxis(3);	
		
		pressurer = rightpressure.getVoltage(); // get the voltage from the force-sensitive resistors
		pressurel = leftpressure.getVoltage();
		
		limit = balllimit.get();
		
		drive(leftstick,rightstick); // drive function
		
		manageGears(pressurel, pressurer, buttonl); // gearshift logic code
		
		manageBall(triggerl-triggerr,limit); // ball logic and timing code
		
		sendData(); // periodic function
	}
	
	/* the following useful functions:
	 * 
	 * drive(double left_joystick, double right_joystick) : moves the motors, handles deadzone and ideally reversing stuff
	 * manageGears(double analog_pressure_left, double analog_pressure_right, boolean override_switch) : automatic gearshifting with manual override
	 * manageBall(double direction) : moves the ball in or out (probably just a motor controller)
	 */
	
	//////////////////////////////// various management functions
	
	void drive(double left, double right) {
		if (Math.abs(left) > Settings.get("deadzonesp")) { // deadzone the motors
			leftdrive.set(left);
		} else {
			leftdrive.set(0);
		}
		
		if (Math.abs(right) > Settings.get("deadzonesp")) {
			rightdrive.set(right);
		} else {
			rightdrive.set(0);
		}
	}
	
	boolean timerRunning = false; // for some reason, timers don't have a field to figure this out
	boolean intake = false;
	boolean eject = false;
	
	void manageBall(double direction, boolean limit) {
		// positive values eject, negative values intake
		// ideally ballholder.forward will open it
		if (direction < -Settings.get("deadzonesp")) { // if the ball is coming in
			openServos(); // open servos to accept the ball
			ballholder.forward(); // drop the plate
			ballmotor.set(direction); // spin up the motor in reverse to intake
			intake = true; // set these values 
			eject = false;
		} else if (direction > Settings.get("deadzonesp")) {
			ballmotor.set(direction); // spin up the motor to eject
			if (!timerRunning) { // to time the ball holder: it needs to spin up the motors before the holder drops the ball for maximum speed ejection
				ballTimer.start();
				timerRunning = true;
			}
			eject = true; // set these
			intake = false;
		} else {
			ballmotor.set(0); // stop the motor, and stop the timers (TODO: fix)
			timerRunning = false;
			ballTimer.stop();
			eject = false;
			intake = false;
		}
		//// for the eject mode, we want to spin up the motors and wait a sec to eject the ball
		if (timerRunning && eject && ballTimer.get() > Settings.get("balltimersp")) { // if waiting to eject, and time has passed
			openServos(); // open holder servos
			ballholder.forward(); // drop the plate
			ballTimer.stop(); // stop and reset timer
			ballTimer.reset();
			timerRunning = false;
		}
		//// for intake mode, we need to wait until the limit is triggered and then lift the plate, and then wait a sec and close the servos
		if (timerRunning && intake && limit && ballTimer.get() > Settings.get("balltimersp")) {
			closeServos(); // close the servos after the timer has expired
			ballTimer.stop(); // and reset the variables
			ballTimer.reset();
			timerRunning = false;
		} else if (intake && limit) { // if limit switch is tripped and intaking
			if (!timerRunning) { 
				ballTimer.start(); // if timer hasn't started, start it
				timerRunning = true;
				ballholder.backward(); // and lift the plate as soon as the limit is tripped
			}
		}
	}
	
	boolean lastgear = true; // true = high, false = low gear
	
	void manageGears(double lpressure, double rpressure, boolean override) {
		// not sure what to do with the encoder/speed thing yet (double lencoder, double rencoder, double lspeed, double rspeed,) <-- add that in
		boolean gear = !(lpressure > Settings.get("pressuresp") || rpressure > Settings.get("pressuresp") || override);

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
		double a_range = range.getVoltage() / (5.0/1024.0); // centimeters hopefully
		rangefinder.newSample(a_range);
		
		TimedDoubleSolenoid.run(); // run all the solenoids
		
		Settings.sync();
		
		syncSensors();
		
		//digit board
		try {
			disp.display(ds.getBatteryVoltage()); //live voltage readout ideally, or whatever we need
		} catch (RuntimeException a) { } // to handle I2C errors?
		
		// now do light stuff
		runLights();
		
		// manage camera MUX
		//switchCamera();
	}
	
	void runLights() {
		try {
			if (arduinoTimer.get() > Settings.get("arduinotimersp")) { // send every 200msec to avoid buffer overflows
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
		//NIVision.IMAQdxStopAcquisition(cameras[curcamera]); // stop current camera (might actually not want to start/stop so often
		curcamera += 1; curcamera %= numcameras; // advance counter
		//NIVision.IMAQdxConfigureGrab(cameras[curcamera]); // start new camera (disabled)
		NIVision.IMAQdxGrab(cameras[curcamera], curframe, 1); // copy the image
		cserver.setImage(curframe); // I hope this doesn't lag anything
	}
	
	void syncSensors() {
		try { // put data into table (probably disable this during comp
			table.putNumber("pwr_v",pdp.getVoltage()); // PDP voltage (not the same as DS voltage)
			table.putNumber("pwr_t",pdp.getTemperature()); // useful to tell if there are things heating up
			table.putNumber("pwr_c",pdp.getTotalCurrent()); // total current draw
			for (int i = 0; i < 16; i++) 
				table.putNumber("pwr_c_" + i,pdp.getCurrent(i)); // current draw for all 16 channels
			table.putNumber("pcm_c",pcm.getCompressorCurrent()); // compressor current draw
			//table.putNumber("range",a_range); // rangefinder raw
			table.putNumber("range_smooth",rangefinder.getAverage()); // averaged rangefinder value
		} catch (RuntimeException a) { } // runtime exception could be caused by CAN timeout
	}
}
