package org.usfirst.frc.team3216.robot;

import edu.wpi.first.wpilibj.Preferences;
import java.util.HashMap;
import java.util.Map;

public class Settings {
	/// static stuff
	//static ArrayList<Settings> settings = new ArrayList<Settings>();
	static Map<String,Settings> settings = new HashMap<String,Settings>(); // hold a list of settings
	static Preferences pref = Preferences.getInstance(); // combines nonvolatle memory and networktables
	
	static void sync() {
		for (Settings s:settings.values()) { // sync all the settings
			s._sync();
		}
	}
	
	static void add(String name, double def) { // add a setting with the default value
		Settings temp = new Settings(name,def);
		settings.put(name, temp);
	}
	
	static double get(String name) { // get a setting
		Settings temp = settings.get(name);
		if (temp == null) {
			return 0; // not in the array = 0 i guess
		} else {
			return temp.value;
		}
	}
	
	/// dynamic stuff
	double value; // all the values are doubles for ease of use
	double defv; // default value
	String name; // name of setting
	
	Settings(String name, double def) { // this isn't supposed to be publicly instantiated
		this.name = name;
		this.defv = def;
		if (!pref.containsKey(name)) {
			pref.putDouble(name,defv);
		}
		this._sync();
	}
	
	void _sync() {
		double temp = pref.getDouble(this.name, this.defv);
		if (this.value != temp) this.value = temp;
	}
}
