package org.usfirst.frc.team3216.robot;
// all them imports:
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import org.usfirst.frc.team3216.robot.REVDigitBoard;
import org.usfirst.frc.team3216.robot.Arduino;
import org.usfirst.frc.team3216.robot.MovingAverage;

public class Robot extends IterativeRobot {
	// setting up all the objects
	// global objects:
	Joystick xBox; // xbox controller, obvoiusly (well actually a logitech gamepad)
	
	Compressor pcm; // pneumatics compressor and solenoid controller
	PowerDistributionPanel pdp; // to get voltage/amperage stuff
	DriverStation ds; // getting DS state, other info
	NetworkTable table; // sending data back & forth for sensors
	Preferences pref; // stateful settings (nonvolatile)
	
	VictorSP leftdrive,rightdrive; // y-cable thiede outputs to the two speed controllers (2 motors per side)
	DoubleSolenoid shift; // connect the two shifter pneumatic actuators in parallel
	Talon ballmotor; // motor to push/pull ball in and out
	DoubleSolenoid ballholder; // two parallel pneumatic actuators to lift ball and hold it in place
	
	AnalogInput range, leftpressure, rightpressure; // range = analog out of ultrasonic rangefinder, *pressure = analog pressure resistors on the front
	Timer arduinoTimer, gearTimer, ballTimer; // timers for handling timed events
	
	REVDigitBoard disp; // digit board connected to MXP
	//SerialPort arduino; // this never worked
	Arduino arduino; // wrapper class to make I2C setup easier
	MovingAverage rangefinder; // smooth spikes in the rangefinder input by averaging the last several samples
	
	static boolean autonstart = false, autonend = false; // to help handle the 3-stage auton: startup, periodic, end
	
	// setpoints
	static double pressuresp = 10; // setpoint for pressure sensors to trigger the automatic low gear switch
	static int autondistsp = 250; // distance to move in autonomous (cm iirc)
	static double deadzonesp = 0.15; // deadzone in joysticks
	static double triggerdzsp = 0.15; // trigger deadzone
	static double geartimersp = 0.3; // timer for gearshift delay (in seconds)
	static double arduinotimersp = 0.2; // send data to arduino every x seconds
	
	/* Connections:
	 * 
	 * left motors: pwm 0
	 * right motors: pwm 1
	 * gearshift: solenoid 0&1
	 * ball holder: solenoid 2&3
	 * rangefinder: analog input 0
	 * pressure sensors on front: analog 2&3
	 * onboard i2c to an arduino
	 */
	
	public void robotInit() {

		xBox = new Joystick(0); // joystick port 0
		pcm = new Compressor(0); // compressor on can network at id 0
		table = NetworkTable.getTable("datatable"); // this table communicates back to the computer for diagnostic purposes
		pdp = new PowerDistributionPanel(); // pdp objecto to read amperages, etc.
		ds = DriverStation.getInstance(); // to get match info for LEDs
		pref = Preferences.getInstance(); // to get/set setpoints and other nonvolatile data
		
		leftdrive = new VictorSP(0); // left motors = pwm 0
		rightdrive = new VictorSP(1); // right motors = pwm 1
		
		shift = new DoubleSolenoid(0,1); // shifter pneumtics on solenoid 0,1
		
		ballmotor = new Talon(2); // ball suck/eject motor = pwm 2
		ballholder = new DoubleSolenoid(2,3); // ball holder pneumatics on solenoid 2,3
		
		range = new AnalogInput(0); // analog rangefinder
		rangefinder = new MovingAverage(5,250); // moving average for rangefinder
		
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
		
		autonstart = false; // make sure these are false for the third time
		autonend = false;
		
		/// prefs:  (there are sooo many better ways to do this... in python)
		if (!pref.containsKey("pressuresp")) { // if key doesn't exist...
			pref.putDouble("pressuresp", pressuresp); // add the setpoint from ram (hard-coded)
		}
		if (!pref.containsKey("autondistsp")) {
			pref.putInt("autondistsp", autondistsp);
		}
		if (!pref.containsKey("deadzonesp")) {
			pref.putDouble("deadzonesp", deadzonesp);
		}
		if (!pref.containsKey("triggerdzsp")) {
			pref.putDouble("triggerdzsp", triggerdzsp);
		}
		if (!pref.containsKey("arduinotimersp")) {
			pref.putDouble("arduinotimersp",arduinotimersp);
		}
		if (!pref.containsKey("geartimersp")) {
			pref.putDouble("geartimersp",geartimersp);
		}
	}

	// This function is called periodically during autonomous
	public void autonomousPeriodic() {
		sendData(); // this also does the moving average stuff
		
		if (!autonstart) {
			// TODO: set up the stuff
			autonstart = true;
		} else if (autonstart && !autonend) {
			double speed = 0.5; //map(a_range, 0, 200, 0, 1)*0.5+0.1; // this was a variable speed formula but it didn't work
			if (rangefinder.getAverage() < autondistsp) {
				//  TODO: drive forward
			} else {
				autonend = true;
			}
		} else if (autonend) {
			// TODO: stop it
		}
	}

	// variables used in teleop:
	double leftstick, rightstick, triggerr, triggerl, pressurer, pressurel; // these are data points pulled off the xbox controller
	boolean buttonl, buttonr; // also xbox controller stuff
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
		
		drive(leftstick,rightstick); // drive function
		
		manageGears(pressurel, pressurer, buttonl); // gearshift logic code
		
		manageBall(triggerl-triggerr); // ball logic and timing code
		
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
		if (Math.abs(left) > deadzonesp) { // deadzone the motors
			leftdrive.set(left);
		} else {
			leftdrive.set(0);
		}
		
		if (Math.abs(right) > deadzonesp) {
			rightdrive.set(right);
		} else {
			rightdrive.set(0);
		}
	}
	
	boolean timerRunning = false; // for some reason, timers don't have a field to figure this out
	
	void manageBall(double direction) {
		// positive values eject, negative values take in
		
		if (Math.abs(direction) > deadzonesp) { // deadzone this as well
			rightdrive.set(direction);
			if (!timerRunning) { // to time the ball holder: it needs to spin up the motors before the holder drops the ball for maximum speed ejection`
				ballTimer.reset();
				ballTimer.start();
				timerRunning = true;
			}
		} else {
			rightdrive.set(0); // stop it all
			timerRunning = false;
		}
	}
	
	boolean lastgear = true; // true = high, false = low gear
	
	void manageGears(double lpressure, double rpressure, boolean override) {
		// not sure what to do with the encoder/speed thing yet (double lencoder, double rencoder, double lspeed, double rspeed,) <-- add that in
		boolean gear = !(lpressure > pressuresp || rpressure > pressuresp || override);

		// this will stop the solenoid after a time
		if (gearTimer.get() > geartimersp) { 
			shift.set(DoubleSolenoid.Value.kOff);
			gearTimer.stop();
			gearTimer.reset();
			lastgear = gear;
		}
		else if (gear != lastgear && gear) { // if change is needed
			shift.set(DoubleSolenoid.Value.kForward);
			gearTimer.start();
		}
		else if (gear != lastgear && !gear) { 
			shift.set(DoubleSolenoid.Value.kReverse);
			gearTimer.start();
		}
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
	
		try { // put data into table (probably disable this during comp
			table.putNumber("pwr_v",pdp.getVoltage()); // PDP voltage (not the same as DS voltage)
			table.putNumber("pwr_t",pdp.getTemperature()); // useful to tell if there are things heating up
			table.putNumber("pwr_c",pdp.getTotalCurrent()); // total current draw
			for (int i = 0; i < 16; i++) 
				table.putNumber("pwr_c_" + i,pdp.getCurrent(i)); // current draw for all 16 channels
			table.putNumber("pcm_c",pcm.getCompressorCurrent()); // compressor current draw
			table.putNumber("range",a_range); // rangefinder raw
			table.putNumber("range_smooth",rangefinder.getAverage()); // averaged rangefinder value
		} catch (RuntimeException a) { } // runtime exception could be caused by CAN timeout
		
		// settings... soooo much work
		double temp = pref.getDouble("pressuresp", pressuresp); // if changed in networktable, update here
		if (pressuresp != temp) pressuresp = temp;
		int temp2 = pref.getInt("autondistsp", autondistsp);
		if (autondistsp != temp2) autondistsp = temp2;
		temp = pref.getDouble("deadzonesp", deadzonesp);
		if (deadzonesp != temp) deadzonesp = temp;
		temp = pref.getDouble("triggerdzsp", triggerdzsp);
		if (triggerdzsp != temp) triggerdzsp = temp;
		temp = pref.getDouble("arduinotimersp", arduinotimersp);
		if (arduinotimersp != temp) arduinotimersp = temp;
		temp = pref.getDouble("geartimersp", geartimersp);
		if (geartimersp != temp) geartimersp = temp;
		// the processing program on the other end will edit the "Preferences" networktable live
		
		//digit board
		try {
			disp.display(ds.getBatteryVoltage()); //live voltage readout ideally, or whatever we need
		} catch (RuntimeException a) { } // to handle I2C errors?
		
		// now do light stuff
		try {
			if (arduinoTimer.get() > arduinotimersp) { // send every 200msec to avoid buffer overflows
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
}
