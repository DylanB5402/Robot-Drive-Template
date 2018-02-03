/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team687.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	public Joystick lStick = new Joystick(0);
	public Joystick rStick = new Joystick(1);
	
	public OI() {
		
		
	}
	
	public double getLeftY() {
		return lStick.getY();
	}
	
	public double getRightY() {
		return rStick.getY();
	}
	public double getRightX() {
		return rStick.getX();
	}
	public void reportToSmartDashboard() {
		SmartDashboard.putNumber("Left Stick Y", getLeftY());
		SmartDashboard.putNumber("Right Stick Y", getRightY());
	}
}
