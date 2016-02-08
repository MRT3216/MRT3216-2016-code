package org.usfirst.frc.team3216.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.*;

/*
 *  the idea with this class is to open a solenoid for long enough to let the air flow,
 *  but no waste air by leaving it open for a while
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
	
	TimedDoubleSolenoid(int a, int b) {
		this.solenoid = new DoubleSolenoid(a,b);
		this.timer = new Timer();
	}
	
	void forward() {
		this.solenoid.set(DoubleSolenoid.Value.kForward);
		this.timer.start();
	}
		
	void backward() {
		this.solenoid.set(DoubleSolenoid.Value.kReverse);
		this.timer.start();
	}
	
	void _run() {
		if (this.timer.get() > Settings.get("solenoidtimersp")) { 
			solenoid.set(DoubleSolenoid.Value.kOff);
			this.timer.stop();
			this.timer.reset();
		}
	}
}
