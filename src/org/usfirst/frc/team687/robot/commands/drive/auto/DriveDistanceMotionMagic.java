package org.usfirst.frc.team687.robot.commands.drive.auto;

import org.usfirst.frc.team687.robot.Robot;
import org.usfirst.frc.team687.robot.constants.DriveConstants;
import org.usfirst.frc.team687.robot.utilities.NerdyMath;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveDistanceMotionMagic extends Command {

	private double m_distance;
	private double m_error;
	
    public DriveDistanceMotionMagic(double distance) {
    	m_distance = distance;
       requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	m_error = m_distance - Robot.drive.getAverageEncoderPosition();
    	Robot.drive.setPositionMotionMagic(m_distance, m_distance);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return NerdyMath.errorTolerance(m_error, DriveConstants.kDriveTolerance);
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive.setPowerZero();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
