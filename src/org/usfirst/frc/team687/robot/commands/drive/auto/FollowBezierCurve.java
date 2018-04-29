package org.usfirst.frc.team687.robot.commands.drive.auto;

import org.usfirst.frc.team687.robot.Robot;
import org.usfirst.frc.team687.robot.constants.DriveConstants;
import org.usfirst.frc.team687.robot.utilities.BezierCurve;
import org.usfirst.frc.team687.robot.utilities.NerdyMath;

import edu.wpi.first.wpilibj.command.Command;

/**
 *Tedklin's DriveBezierPath adapted to use the robot's pose instead of encoder and yaw
 */
public class FollowBezierCurve extends Command {
	
//	m_b_____ = x-intercept of line
//	m_m_____ = slope of line 
	
	private BezierCurve m_curve;
	private double m_straightPower;
	private boolean m_useStraightPID;
	private double m_currentX;
	private double m_currentY;
	private int m_t;
	private double m_desiredX;
	private double m_desiredY;
	private double m_distanceToPoint;
	private double m_desiredAngle;
	private double m_rotationalError;
	private double m_rotationalPower;
	private double m_angle;
	private double m_straightError;
	
    public FollowBezierCurve(BezierCurve curve, double straightPower, double lookahead, boolean useStraightPID) {
    	m_curve = curve;
    	m_straightPower = straightPower;
    	m_useStraightPID = useStraightPID;
        requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	m_currentX = Robot.drive.getXpos();
    	m_currentY = Robot.drive.getYpos();
    	m_t = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	m_desiredX = m_curve.getX(m_t);
    	m_desiredY = m_curve.getY(m_t);
    	m_distanceToPoint = NerdyMath.distanceFormula(m_currentX, m_currentY, m_desiredX, m_currentY);
    	m_desiredAngle = Math.atan2(m_desiredX - m_currentX, m_desiredY - m_currentY);
    	m_angle = (360 - Robot.drive.getRawYaw()) % 360;
    	m_rotationalError = -m_desiredAngle - m_angle;
    	if (m_rotationalError >= 180) {
    		m_rotationalError -= 360;
    	}
    	if (m_rotationalError <= -180) {
    		m_rotationalError += 360;
    	}
    	m_rotationalPower = m_rotationalError * DriveConstants.kRotP;   	
    	if (m_useStraightPID) {
    		m_straightError = m_curve.getCurveLength() - (Robot.drive.getLeftMasterPosition() + Robot.drive.getRightMasterPosition())/2;
        	m_straightPower = m_straightError * DriveConstants.kDriveP;
    	}  
    	Robot.drive.setPower(m_straightPower - m_rotationalPower, m_straightPower + m_rotationalPower);    	
    	if (m_distanceToPoint < DriveConstants.kMinDistToBezierPoint) {
    		m_t += 1;
    	}   	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (m_curve.getCurveLength() - (Robot.drive.getLeftMasterPosition() + Robot.drive.getRightMasterPosition())/2) < DriveConstants.kMinDistToBezierPoint;
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