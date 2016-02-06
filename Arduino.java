package org.usfirst.frc.team3216.robot;

import edu.wpi.first.wpilibj.I2C;

public class Arduino { // simple wrapper class to help with single-byte arduino I2C comms
	I2C i2c;
	
	Arduino(byte addr) {
		i2c = new I2C(I2C.Port.kOnboard,addr); // onboard i2c is the one next to the power in on the roborio
	}
	void sendbyte(byte a) {
		byte[] b = {a}; // make an array, because java
		i2c.transaction(b, 1, null, 0); // send the byte
	}
}
