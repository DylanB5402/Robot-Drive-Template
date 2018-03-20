package org.usfirst.frc.team687.robot.commands;

import org.usfirst.frc.team687.robot.Robot;
import org.usfirst.frc.team687.robot.constants.DriveConstants;
import org.usfirst.frc.team687.robot.utilities.NerdyMath;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnToAngle extends Command {
	
	private double m_desiredAngle;
	private double m_error;
	private double m_currentAngle;
	private double m_power;
	
    public TurnToAngle(double angle) {
    	m_desiredAngle = angle;
        requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	m_currentAngle = Robot.drive.getAngle();
    	m_error = -m_desiredAngle - m_currentAngle;
    	
    	if (m_error >= 180) {
    		m_error -= 360;
    	}
    	if (m_error <= -180) {
    		m_error += 360;
    	}
    	
    	m_power = m_error * DriveConstants.kRotP;
    	
    	NerdyMath.threshold(m_power, DriveConstants.kMinRotPower, DriveConstants.kMaxRotPower);
    	
    	Robot.drive.setPower(m_power, m_power);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(m_error) <= DriveConstants.kDriveRotationTolerance;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive.setPowerZero();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
