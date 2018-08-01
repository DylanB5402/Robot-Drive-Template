package org.usfirst.frc.team687.robot.commands.drive.auto;

import org.usfirst.frc.team687.robot.Robot;
import org.usfirst.frc.team687.robot.constants.DriveConstants;
import org.usfirst.frc.team687.robot.utilities.BezierCurve;
import org.usfirst.frc.team687.robot.utilities.NerdyMath;

import edu.wpi.first.wpilibj.command.Command;

/**
 *Tedklin's DriveBezierPath adapted to use the robot's pose instead of encoder and yaw
 */
public class DriveBezierCurve extends Command {
	
	
	private BezierCurve m_path;
	private double m_straightPower, m_currentX, m_currentY, m_desiredX, m_desiredY, m_distanceToPoint;
	private double m_desiredAngle, m_rotationalError, m_rotationalPower, m_angle, m_straightError, m_direction;
	private int m_t;
	private boolean m_useStraightPID;
	/**
	 * 
	 * @param path = Bezier Curve to follow
	 * @param straightPower = positive is forwards, negative for backwards
	 * @param useStraightPID =
	 */
	
    public DriveBezierCurve(BezierCurve path, double straightPower, boolean useStraightPID) {
    	m_path = path;
    	m_straightPower = straightPower;
    	m_useStraightPID = useStraightPID;
        requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	m_currentX = Robot.drive.getXpos();
    	m_currentY = Robot.drive.getYpos();
    	m_t = 0;
    	m_direction = Math.signum(m_straightPower);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	m_currentX = Robot.drive.getXpos();
    	m_currentY = Robot.drive.getYpos();
    	m_desiredX = m_path.getX(m_t);
    	m_desiredY = m_path.getY(m_t);
    	m_distanceToPoint = NerdyMath.distanceFormula(m_currentX, m_currentY, m_desiredX, m_currentY);
    	m_desiredAngle = Math.atan2(m_desiredX - m_currentX, m_desiredY - m_currentY);
    	if (m_direction == -1) {
    		m_desiredAngle += 180;
    	}
    	m_angle = (360 - Robot.drive.getRawYaw()) % 360;
    	m_rotationalError = -m_desiredAngle - m_angle;
    	if (m_rotationalError > 180) {
    		m_rotationalError -= 360;
    	}
    	if (m_rotationalError < -180) {
    		m_rotationalError += 360;
    	}
    		
    	m_rotationalPower = m_rotationalError * DriveConstants.kRotP;   	
    	
    	if (m_useStraightPID) {
    		m_straightError = m_path.getCurveLength() - Robot.drive.getAverageEncoderPosition();
        	m_straightPower = m_straightError * DriveConstants.kDriveP;
    	}  
    	else {
    		m_straightPower = Math.copySign(m_straightPower, m_direction);
    	}
    	Robot.drive.setPower(m_straightPower - m_rotationalPower, m_straightPower + m_rotationalPower);    	
    	if (m_distanceToPoint < DriveConstants.kMinDistToBezierPoint) {
    		m_t += 1;
    	}   	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return m_desiredX == m_path.getLastX() && m_desiredY == m_path.getLastY() && (NerdyMath.distanceFormula(m_currentX, m_currentY, m_path.getLastX(), m_path.getLastY()) < DriveConstants.kMinDistToBezierPoint);
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
