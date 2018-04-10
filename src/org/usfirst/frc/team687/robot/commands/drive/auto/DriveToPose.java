package org.usfirst.frc.team687.robot.commands.drive.auto;

import org.usfirst.frc.team687.robot.Robot;
import org.usfirst.frc.team687.robot.constants.DriveConstants;
import org.usfirst.frc.team687.robot.utilities.NerdyMath;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveToPose extends Command {

	private double m_desiredX;
	private double m_desiredY;
	private double m_desiredAngle;
	private double m_straightError;
	private double m_rotationalError;
	private double m_straightPower;
	private double m_rotationalPower;
	
    public DriveToPose(double x, double y, double angle) {
    	m_desiredX = x;
    	m_desiredY = y;
    	m_desiredAngle = angle;
        requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	m_straightError = NerdyMath.distanceFormula(Robot.drive.getXpos(), Robot.drive.getYpos(), m_desiredX, m_desiredY);
    	m_rotationalError = -m_desiredAngle - Robot.drive.getAngle();
    	
    	if (m_rotationalError >= 180) {
    		m_rotationalError -= 360;
    	}
    	if (m_rotationalError <= -180) {
    		m_rotationalError += 360;
    	}
    	
    	m_straightPower = m_straightError * DriveConstants.kDriveP;
    	m_rotationalPower = m_rotationalError * DriveConstants.kRotP;
    	m_straightPower = NerdyMath.threshold(m_straightPower, 0, 0.5);
    	m_rotationalPower = NerdyMath.threshold(m_rotationalPower, 0, 0.5);
    	Robot.drive.setPower(m_straightPower - m_rotationalPower, m_straightPower + m_rotationalPower);
    	 	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return NerdyMath.errorTolerance(m_straightError, DriveConstants.kDriveTolerance) && NerdyMath.errorTolerance(m_rotationalError, m_rotationalError);
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
