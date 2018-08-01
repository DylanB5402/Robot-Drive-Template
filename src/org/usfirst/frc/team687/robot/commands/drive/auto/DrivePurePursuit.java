package org.usfirst.frc.team687.robot.commands.drive.auto;

import org.usfirst.frc.team687.robot.Robot;
import org.usfirst.frc.team687.robot.constants.DriveConstants;
import org.usfirst.frc.team687.robot.utilities.BezierCurve;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *Implementation of the pure pursuit algorithm for path following
 *https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
 *https://www.desmos.com/calculator/z6lujl7js2
 *https://www.chiefdelphi.com/media/papers/3398
 *
 *INCOMPLETE/WIP
 */
public class DrivePurePursuit extends Command {

	private BezierCurve m_path;
	private double m_cruiseVel, m_accel, m_dist, m_robotX, m_robotY, m_slope, m_yInt, m_a, m_b, m_c;
	private double m_goalX, m_goalY, m_outerVel, m_innerVel, m_lookahead;
	private double m_startingDist, m_startTime, m_time;
	private double m_accelTime, m_maxAccelDist, m_maxAccelTime, m_cruiseDist, m_cruiseTime, m_vel;
	private double m_x1, m_x2, m_y1, m_y2, m_driveRadius;
	private int m_t;
	private boolean m_goingForward;
    public DrivePurePursuit(BezierCurve path, double cruiseVel, double accel, double lookahead, boolean goingForward) {
    	m_path = path;
    	m_cruiseVel = cruiseVel;
    	m_accel = accel;
    	m_lookahead = lookahead;
    	m_goingForward = goingForward;
        requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	m_dist = m_path.getCurveLength();
    	m_t = 0;
    	m_startTime = Timer.getFPGATimestamp();
    	m_maxAccelTime = m_cruiseVel/m_accel;
    	m_maxAccelDist = 0.5 * m_accel * Math.pow(m_maxAccelTime, 2);
    	m_accelTime = Math.sqrt((2*m_maxAccelDist)/m_accel);
		m_cruiseDist = m_dist - 2 * m_maxAccelDist;
		m_cruiseTime = (m_cruiseDist/m_cruiseVel) + m_accelTime; 
		
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	m_time = Timer.getFPGATimestamp() - m_startTime;
    	m_robotX = Robot.drive.getXpos();
    	m_robotY = Robot.drive.getYpos();
    	m_x1 = m_path.getX(m_t);
    	m_y1 = m_path.getY(m_t);
    	m_x2 = m_path.getX(m_t + 1);
    	m_y2 = m_path.getY(m_t + 1);
    	m_slope = (m_y2-m_y1)/(m_x2-m_x1);
    	m_yInt = m_y2 - m_slope * m_x2;
    	//quadratic formula to determine where lookahead circle from robot and line segment to follow intersect
    	m_a = (1 + Math.pow(m_slope, 2));
    	m_b = -2 * m_robotX + 2 * m_slope * (m_yInt - m_robotY);
    	m_c = Math.pow(m_robotX, 2) + Math.pow((m_yInt - m_robotY), 2) - Math.pow(m_lookahead, 2);
    	if ((Math.signum(m_slope) == 1 && m_goingForward) || (Math.signum(m_slope) == -1 && !m_goingForward)) {
    		m_goalX = (-m_b + Math.sqrt(Math.pow(m_b, 2) - 4 * m_a * m_c))/(2 * m_a);
    	}
    	else if ((Math.signum(m_slope) == -1 && m_goingForward) || (Math.signum(m_slope) == 1 && !m_goingForward)) {
    		m_goalX = (-m_b - Math.sqrt(Math.pow(m_b, 2) - 4 * m_a * m_c))/(2 * m_a);
    	}
    	m_goalY = m_slope * m_goalX + m_yInt;
    	m_driveRadius = (Math.pow(m_lookahead, 2)/ (2 * (m_goalX - m_robotX)));
    	
    	if (m_time <= m_accelTime) {
			m_vel = m_accel * m_time;
		}
		else if (m_time > m_accelTime && m_time < m_cruiseTime) {
			m_vel = m_cruiseVel;
		}
		else if (m_time >= m_cruiseTime) {
			m_vel = -m_accel * (m_time - m_accelTime - m_cruiseTime);
		}
    	m_outerVel = m_vel;
    	m_innerVel = m_outerVel * (m_driveRadius - (DriveConstants.kDrivetrainWidth/2)) /( m_driveRadius - (DriveConstants.kDrivetrainWidth/2));
    	if (Math.signum(m_driveRadius) > 0) {
    		Robot.drive.setVelocity(m_innerVel, m_outerVel);
    	}
    	else {
    		Robot.drive.setVelocity(m_outerVel, m_innerVel);
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
