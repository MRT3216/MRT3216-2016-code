package org.usfirst.frc.team3216.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.*;

/*
 *  the idea with this class is to open a solenoid for long enough to let the air flow,
 *  but not waste air by leaving it open for a while
 */

public class TimedDoubleSolenoid { 
	// shared stuff
	static ArrayList<TimedDoubleSolenoid> solenoids = new ArrayList<TimedDoubleSolenoid>();
	
	static void run() {
		for (TimedDoubleSolenoid s: solenoids) {
			s._run();
		}
	}
	
	// instantiated stuff
	DoubleSolenoid solenoid;
	Timer timer;
	boolean running;
	
	TimedDoubleSolenoid(int a, int b) { // init with the two solenoid channels 
		this.solenoid = new DoubleSolenoid(a,b);
		this.timer = new Timer();
		this.running = false;
		//solenoids.add(this);
	}
	
	void forward() {
		this.solenoid.set(DoubleSolenoid.Value.kForward);
		this.timer.reset();
		this.timer.start();
		this.running = true;
	}
		
	void backward() {
		this.solenoid.set(DoubleSolenoid.Value.kReverse);
		this.timer.reset();
		this.timer.start();
		this.running = true;
	}
	
	void _run() {
		if (this.running && this.timer.get() > Settings.get("solenoidtimersp")) { // global timer setpoint
			solenoid.set(DoubleSolenoid.Value.kOff);
			this.timer.stop();
			this.timer.reset();
			this.running = false;
		}
	}
}
