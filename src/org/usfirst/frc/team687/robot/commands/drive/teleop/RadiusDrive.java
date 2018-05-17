package org.usfirst.frc.team687.robot.commands.drive.teleop;

import org.usfirst.frc.team687.robot.Robot;
import org.usfirst.frc.team687.robot.constants.DriveConstants;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Implementation of 3184's radius turn, based off of this whitepaper
 * https://www.chiefdelphi.com/media/papers/3398
 * Planning on implementing this for pure pursuit sometime in the next 50000 years
 * (we'll have pure pursuit when there's a water game)
 */
public class RadiusDrive extends Command {

	private double m_turnPower;
	private double m_straightPower;
	private double m_innerPower;
	private double m_turnRadius;
	
    public RadiusDrive() {
        requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	m_turnPower = Robot.oi.getLeftX();
    	m_straightPower = Robot.oi.getRightY();
    	if (Math.abs(m_turnPower) < DriveConstants.kJoystickDeadband) {
    		Robot.drive.setPower(m_straightPower, m_straightPower);
    	}
    	m_turnRadius = DriveConstants.kMaximumTurnRadius * (1 - Math.abs(m_turnPower));
    	m_innerPower = m_straightPower * (m_turnRadius - DriveConstants.kDrivetrainWidth/2)/(m_turnRadius + DriveConstants.kDrivetrainWidth/2);
    	if (m_turnPower > 0) {
    		Robot.drive.setPower(m_straightPower, m_innerPower);
    	}
    	else if (m_turnPower < 0) {
    		Robot.drive.setPower(m_innerPower, m_straightPower);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
