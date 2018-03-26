package org.usfirst.frc.team687.robot.commands.drive.auto;

import org.usfirst.frc.team687.robot.Robot;
import org.usfirst.frc.team687.robot.constants.DriveConstants;
import org.usfirst.frc.team687.robot.utilities.NerdyMath;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnToAngleMotionMagic extends Command {

	private double m_desiredAngle;
	private double m_error;
	private double m_DesiredEncoderPosition;
	
    public TurnToAngleMotionMagic(double angle) {
    	m_desiredAngle = angle;
       requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drive.resetEncoders();
    	m_error = -m_desiredAngle - Robot.drive.getAngle();
    	
    	if (m_error >= 180) {
    		m_error -= 360;
    	}
    	if (m_error <= -180) {
    		m_error += 360;
    	}
    	
    	m_DesiredEncoderPosition = NerdyMath.angleToTicks(m_error);
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() { 
    	m_error = -m_desiredAngle - Robot.drive.getAngle();
    	Robot.drive.resetEncoders();
    	Robot.drive.setPositionMotionMagic(m_DesiredEncoderPosition, -m_DesiredEncoderPosition);    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return NerdyMath.errorTolerance(m_error, DriveConstants.kDriveRotationTolerance);
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
