package org.usfirst.frc.team687.robot.commands.drive.teleop;

import org.usfirst.frc.team687.robot.Robot;
import org.usfirst.frc.team687.robot.constants.DriveConstants;
import org.usfirst.frc.team687.robot.utilities.NerdyMath;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Field Relative Drive:
 * Based on standard 2 stick arcade, 1 stick control's robot's turning relative to the field/navx, other controls straight power
 */
public class FieldRelativeDrive extends Command {

	private double m_desiredAngle;
	private double m_error;
	private double m_currentAngle;
	private double m_rotPower;
	private double m_straightPower;
	
    public FieldRelativeDrive() {
        requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	m_desiredAngle = Math.atan2(Robot.oi.getRightY(), Robot.oi.getRightX());
    	m_currentAngle = Robot.drive.getAngle();
    	m_error = -m_desiredAngle - m_currentAngle;
    	
    	if (m_error >= 180) {
    		m_error -= 360;
    	}
    	if (m_error <= -180) {
    		m_error += 360;
    	}
    	
    	m_rotPower = m_error * DriveConstants.kRotP;
    	if (Robot.oi.isLeftTriggerPulled()){
    		m_straightPower = NerdyMath.threshold(Robot.oi.getLeftY(), 0, 0.7);
    	}
    	else {
    		m_straightPower = Robot.oi.getLeftY();
    	}
    	
    	Robot.drive.setPower(m_straightPower - m_rotPower, m_straightPower + m_rotPower);
    	
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
