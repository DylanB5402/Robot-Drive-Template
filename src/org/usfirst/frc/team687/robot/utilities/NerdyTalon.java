package org.usfirst.frc.team687.robot.utilities;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
/**
 * 
 * @author dbarv
 *Basic wrapper for TalonSRXs to make it easier to set PIDFs
 */

public class NerdyTalon extends TalonSRX {

	public NerdyTalon(int talonID) {
		super(talonID);
	}
	
	public void configPIDF(double p, double i, double d, double f, int slot) {
		super.config_kP(slot, p, 0);
		super.config_kI(slot, i, 0);
		super.config_kD(slot, d, 0);
		super.config_kF(slot, f, 0);
	}

	public void configVoltageCompensation(double voltage) {
		super.configVoltageCompSaturation(voltage, 0);
		super.enableVoltageCompensation(true);
	}
	
	public void configCurrentLimit(int current) {
		super.configPeakCurrentLimit(current, 0);
		super.enableCurrentLimit(true);
	}
	
	public void configMotionMagic(int accel, int cruise_vel) {
		super.configMotionAcceleration(accel, 0);
		super.configMotionCruiseVelocity(cruise_vel, 0);
	}
	
	
	
}
