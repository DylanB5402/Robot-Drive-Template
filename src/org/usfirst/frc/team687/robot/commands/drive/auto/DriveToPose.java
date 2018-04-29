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
	private double m_currentX;
	private double m_currentY;
	private boolean m_useStraightPID;
	
    public DriveToPose(double x, double y, double straightPower, boolean useStraightPID) {
    	m_desiredX = x;
    	m_desiredY = y;
    	m_useStraightPID = useStraightPID;
    	m_straightPower = straightPower;
        requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	m_currentX = Robot.drive.getXpos();
    	m_currentY = Robot.drive.getYpos();
    	m_desiredAngle = Math.atan2(m_desiredX - m_currentX, m_desiredY - m_currentY);
    	m_rotationalError = -m_desiredAngle - Robot.drive.getAngle();
    	m_rotationalPower = m_rotationalError * DriveConstants.kRotP;
    	m_rotationalPower = NerdyMath.threshold(m_rotationalPower, 0, 0.5);
    	if (m_rotationalError >= 180) {
    		m_rotationalError -= 360;
    	}
    	if (m_rotationalError <= -180) {
    		m_rotationalError += 360;
    	}
    	
    	if (m_useStraightPID) {
    		m_straightError = NerdyMath.distanceFormula(m_currentX, m_currentY, m_desiredX, m_desiredY);
        	m_straightPower = m_straightError * DriveConstants.kDriveP;
        	m_straightPower = NerdyMath.threshold(m_straightPower, 0, 0.5);
    	}
  
    	Robot.drive.setPower(m_straightPower - m_rotationalPower, m_straightPower + m_rotationalPower);
    	 	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return NerdyMath.distanceFormula(m_currentX, m_currentY, m_desiredX, m_desiredY) < DriveConstants.kMinDistToBezierPoint;
    }

    // Called once after isFinished returns true
    protected void end() {
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
