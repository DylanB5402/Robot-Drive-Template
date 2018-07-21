package org.usfirst.frc.team687.robot.commands.drive.auto;

import org.usfirst.frc.team687.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *Basic 1D motion profile, will later be used for pure pursuit
 */
public class DriveMotionProfile extends Command {

	private double m_cruiseVel, m_accel, m_dist, m_startTime, m_time;
	private double m_accelTime, m_maxAccelDist, m_maxAccelTime, m_cruiseDist, m_cruiseTime, m_vel;
	private boolean m_isTrapezoidal;
	
    public DriveMotionProfile(double cruiseVel, double accel, double distance) {
    	m_cruiseVel = cruiseVel;
    	m_accel = accel;
    	m_dist = distance;
        requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	m_startTime = Timer.getFPGATimestamp();
    	m_maxAccelTime = m_cruiseVel/m_accel;	
    	//d = vt + 1/2at^2
    	m_maxAccelDist = 0.5 * m_accel * Math.pow(m_maxAccelTime, 2);
    			
    	if (m_dist < 2 * m_maxAccelDist) {
    		m_isTrapezoidal = false;
    		m_accelTime = Math.sqrt(m_dist/m_accel);
    	}
    	else if (m_dist > 2 * m_maxAccelDist) {
    		m_accelTime = Math.sqrt((2*m_maxAccelDist)/m_accel);
    		m_isTrapezoidal = true;
    		m_cruiseDist = m_dist - 2 * m_maxAccelDist;
    		m_cruiseTime = (m_cruiseDist/m_cruiseVel) + m_accelTime;  		
    	}
    	    	
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	m_time = Timer.getFPGATimestamp() - m_startTime;
    	if (m_isTrapezoidal) {
    		if (m_time <= m_accelTime) {
    			m_vel = m_accel * m_time;
    		}
    		else if (m_time > m_accelTime && m_time < m_cruiseTime) {
    			m_vel = m_cruiseVel;
    		}
    		else if (m_time >= m_cruiseTime) {
    			m_vel = -m_accel * (m_time - m_accelTime - m_cruiseTime);
    		}
    	}
    	else {
    		if (m_time <= m_accelTime) {
    			m_vel = m_accel * m_time;
    		}
    		else if (m_time > m_accelTime) {
    			m_vel = -m_accel * (m_time - m_accelTime) + (m_accel * m_accelTime);
    		}
    	}
    	m_vel = m_vel * Math.signum(m_dist);
    	Robot.drive.setVelocity(m_vel, m_vel);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return m_time >= m_cruiseTime + m_accelTime;
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
