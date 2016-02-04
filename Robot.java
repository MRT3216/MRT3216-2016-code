package org.usfirst.frc.team3216.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import org.usfirst.frc.team3216.robot.REVDigitBoard;

public class Robot extends IterativeRobot {
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	// global objects:
	Joystick xBox; // xbox controller, obvoiusly
	
	Compressor pcm; // pneumatics compressor
	PowerDistributionPanel pdp;
	DriverStation ds;
	NetworkTable table;
	
	VictorSP leftdrive,rightdrive;
	DoubleSolenoid shift;
	Talon ballmotor;
	DoubleSolenoid ballholder;
	
	AnalogInput range, leftpressure, rightpressure;
	
	REVDigitBoard disp;
	
	SerialPort arduino;
	Timer arduinoTimer, gearTimer, ballTimer;
	
	
	// auton smoothing
	final static int initvalue = 250;
	final static int numreadings = 5;          //The number of samples to smooth.  Helps to avoid incorrect readings due to noise or voltage spikes  
	static double[] readings = new double[numreadings];  //Stores the values from the rangefinder
	static int index   = 0;              //The location within the array to store the value
	static double total = initvalue*numreadings;
	static double r_average = 0;  
	static double a_range;
	
	
	static boolean autonstart = false, autonend = false;
	
	// setpoints
	static double pressuresp = 10; // setpoint for pressure sensors to trigger the automatic low gear switch
	static int autondistsp = 250; // centimeters (411)
	static double deadzonesp = 0.15; // deadzone in joysticks
	static double triggerdzsp = 0.15; // trigger deadzone
	
	/* Connections:
	 * 
	 * left motors: pwm 0
	 * right motors: pwm 1
	 * gearshift: solenoid 0&1
	 * ball holder: solenoid 2&3
	 * rangefinder: analog input 0
	 * pressure sensors on front: analog 2&3
	 */
	
	public void robotInit() {

		xBox = new Joystick(0);
		pcm = new Compressor(0);
		table = NetworkTable.getTable("datatable");
		pdp = new PowerDistributionPanel();
		ds = DriverStation.getInstance();
		
		leftdrive = new VictorSP(0);
		rightdrive = new VictorSP(1);
		
		shift = new DoubleSolenoid(0,1); // easiest way to reverse the controls is to switch these numbers
		
		ballmotor = new Talon(2);
		ballholder = new DoubleSolenoid(2,3);
		
		range = new AnalogInput(0);
		
		leftpressure = new AnalogInput(1);
		rightpressure = new AnalogInput(2);
		
		disp = new REVDigitBoard();
		disp.clear();
		disp.display("3216");
		
		try {
			arduino = new SerialPort(19200,SerialPort.Port.kUSB);
		} catch (RuntimeException a) { }
		
		gearTimer = new Timer();
		arduinoTimer = new Timer();
		arduinoTimer.start();
		ballTimer = new Timer();
		
		pcm.setClosedLoopControl(true); // this is now done programatically by the pneumatics module
		
		for (int i = 0; i < numreadings; i++) readings[i] = initvalue;
		autonstart = false;
		autonend = false;
	}

	// This function is called periodically during autonomous
	public void autonomousPeriodic() {
		sendData(); // this also does the moving average stuff
		
		if (!autonstart) {
			// TODO: start up the stuff
			
			autonstart = true;
		} else if (autonstart && !autonend) {
			double speed = 0.5; //map(a_range, 0, 200, 0, 1)*0.5+0.1;
			
			if (r_average < autondistsp) {
				//  TODO: drive forward
			} else {
				autonend = true;
			}
		} else if (autonend) {
			// stop it
		}
		
	}

	// variables used in teleop:
	double leftstick, rightstick, triggerr, triggerl, pressurer, pressurel;
	boolean buttonl, buttonr;
	// This function is called periodically during operator control
	public void teleopPeriodic() {
		// this took a lot of guess-and-check, since several 
		// axes were messed up this year. don't trust the 
		// joystick explorer, use the DS for testing these
		leftstick = xBox.getRawAxis(1);
		rightstick = xBox.getRawAxis(3);
		
		buttonl = xBox.getRawButton(6); //get button state
		buttonr = xBox.getRawButton(5); 
 
		triggerl = xBox.getRawAxis(2);
		triggerr = xBox.getRawAxis(3);	
		
		pressurer = rightpressure.getVoltage();
		pressurel = leftpressure.getVoltage();
		
		drive(leftstick,rightstick);
		
		manageGears(pressurel, pressurer, buttonl); 
		
		manageBall(triggerl-triggerr);
		
		sendData();
	}
	
	/*
	 * func definitions:
	 * 
	 * drive(double left_joystick, double right_joystick) : moves the motors, handles deadzone and ideally reversing stuff
	 * manageGears(double analog_pressure_left, double analog_pressure_right, boolean override_switch) : automatic gearshifting with manual override
	 * manageBall(double direction) : moves the ball in or out (probably just a motor controller)
	 */
	
	//////////////////////////////// various management functions
	
	void drive(double left, double right) {
		if (Math.abs(left) > deadzonesp) {
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
	
	boolean timerRunning = false;
	
	void manageBall(double direction) {
		// positive values eject, negative values take in
		
		if (Math.abs(direction) > deadzonesp) {
			rightdrive.set(direction);
			if (!timerRunning) ballTimer.start();
			timerRunning = true;
		} else {
			rightdrive.set(0);
			ballTimer.reset();
			timerRunning = false;
		}
	}
	
	boolean lastgear = true; // true = high, false = low gear
	
	void manageGears(double lpressure, double rpressure, boolean override) {
		// not sure what to do with the encoder/speed thing yet (double lencoder, double rencoder, double lspeed, double rspeed,) <-- add that in
		boolean gear = !(lpressure > pressuresp || rpressure > pressuresp || override);

		// this will stop the solenoid after 200 msec 
		if (gearTimer.get() > 0.2) { 
			shift.set(DoubleSolenoid.Value.kOff);
			gearTimer.stop();
			gearTimer.reset();
		}
		else if (gear != lastgear && gear) {
			shift.set(DoubleSolenoid.Value.kForward);
			gearTimer.start();
		}
		else if (gear != lastgear && !gear) {
			shift.set(DoubleSolenoid.Value.kReverse);
			gearTimer.start();
		}
		
		lastgear = gear;
	}

	////////////////////////// miscellaneous stuffs
	
	public void testPeriodic() {
		sendData();
	}
	
	public void disabledPeriodic() {
		sendData();
	}
	
	double map(double value, double istart, double istop, double ostart, double ostop){
		return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
	}
	
	byte[] lastmode = new byte[10];
	
	void sendData() {
		//moving average for rangefinder
		double a_range = range.getVoltage() / (5.0/1024.0); // centimeters
		total -= readings[index];         //subtract the last reading
		readings[index] = a_range;        //place value from sensor
		total += readings[index];         //add the reading to the total
		index++;                          //advance to the next position in the array
		if (index>=numreadings) index=0;  //if its at the end of the array, wrap around to the beginning
		r_average = total / numreadings;    //calculate the average
	
		try {
			// put data into table
			/*table.putNumber("accel_x",acc.getX());
			table.putNumber("accel_y",acc.getY());
			table.putNumber("accel_z",acc.getZ());
			table.putNumber("gyro_r",gyro.getAngle());
			table.putNumber("gyro_s",gyro.getRate());
			table.putNumber("enc_r",winchenc.getRate());
			table.putNumber("pwr_v",pdp.getVoltage());
			table.putNumber("pwr_t",pdp.getTemperature());
			table.putNumber("pwr_c",pdp.getTotalCurrent());
			for (int i = 0; i < 16; i++) 
				table.putNumber("pwr_c_" + i,pdp.getCurrent(i));    
			table.putNumber("pcm_c",pcm.getCompressorCurrent());    */
			table.putNumber("range",a_range);
		} catch (RuntimeException a) { } 
		
		//digit board
		try {
			disp.display(ds.getBatteryVoltage()); //live voltage readout ideally, or whatever we need
		} catch (RuntimeException a) { }
		
		// now do light stuff
		try {
			if (arduino == null)
				arduino = new SerialPort(19200,SerialPort.Port.kUSB);
		} catch (RuntimeException a) { }
		try {
			if (arduinoTimer.get() > 0.2) { // send every 200msec to avoid buffer overflows
				byte mode1 = 0;  //////// 0b<red><blue><fms><auton><teleop><disabled><enabled><attached>
				if (ds.getAlliance() == DriverStation.Alliance.Red)  mode1 |= 0b10000000;
				if (ds.getAlliance() == DriverStation.Alliance.Blue) mode1 |= 0b01000000;
				if (ds.isFMSAttached())                              mode1 |= 0b00100000;
				if (ds.isAutonomous())                               mode1 |= 0b00010000;
				if (ds.isOperatorControl())                          mode1 |= 0b00001000;
				if (ds.isDisabled())                                 mode1 |= 0b00000100;
				if (ds.isEnabled())                                  mode1 |= 0b00000010;
				if (ds.isDSAttached())                               mode1 |= 0b00000001;
				
				byte[] mode2 = {mode1}; // whyyyyyyyyy
				arduino.write(mode2, 1);
				
				arduinoTimer.reset();
				arduinoTimer.start();  //probably don't need to do this
			}
		} catch (RuntimeException a) { }
	}
}
