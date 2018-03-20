package org.usfirst.frc.team687.robot.subsystems;

import org.usfirst.frc.team687.robot.RobotMap;
import org.usfirst.frc.team687.robot.commands.TankDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Drive extends Subsystem {

	private final TalonSRX m_lMasterTalon, m_leftTalon1;
	private final TalonSRX m_rMasterTalon, m_rightTalon1;
	private final AHRS m_nav;
	private double m_previousDistance;
    private double m_currentX, m_currentY;
    
	public Drive() {
		m_lMasterTalon = new TalonSRX(RobotMap.kLeftMasterTalonSRXID);
		m_leftTalon1 = new TalonSRX(RobotMap.kLeftSlaveTalonSRX1ID);
		m_rMasterTalon = new TalonSRX(RobotMap.kRightMasterTalonSRXID);
		m_rightTalon1 = new TalonSRX(RobotMap.kRightSlaveTalonSRX1ID);
		
		m_lMasterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		m_rMasterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		
		m_nav = new AHRS(SPI.Port.kMXP);
		
	}
	
	public void setPower(double leftPower, double rightPower) {

		m_lMasterTalon.set(ControlMode.PercentOutput, leftPower);
		m_leftTalon1.set(ControlMode.PercentOutput, leftPower);
		m_rMasterTalon.set(ControlMode.PercentOutput, rightPower);
		m_rightTalon1.set(ControlMode.PercentOutput, rightPower);
    }
	
	public double getLeftOutputVoltage() {
		return m_lMasterTalon.getMotorOutputVoltage();
	}
	
	public double getLeftMasterCurrent() {
		return m_lMasterTalon.getOutputCurrent();
	}
	
	public double getLeftMasterPosition() {
		return m_lMasterTalon.getSelectedSensorPosition(0);
	}
	
	public double getLeftMasterSpeed() {
		return m_lMasterTalon.getSelectedSensorVelocity(0);
	}
	
	
	public double getRightOutputVoltage() {
		return m_rMasterTalon.getMotorOutputVoltage();
	}
	
	public double getRightMasterCurrent() {
		return m_rMasterTalon.getOutputCurrent();
	}
	
	public double getRightMasterPosition() {
		return m_rMasterTalon.getSelectedSensorPosition(0);
	}
	
	public double getRightMasterSpeed() {
		return m_rMasterTalon.getSelectedSensorVelocity(0);
	}
	
	
	public double getYaw() {
		return m_nav.getYaw();
	}
	
	public void resetYaw() {
		m_nav.reset();
	}
	
	public double getAngle() {
//		converts angle from -180 to 180 to 0 to 360
		
//		sets positive y as 0 deg, robot's front is 0 deg
//		return (360 - getYaw()) % 360;
		
//		sets positive x as 0 deg, robot's front is 90 deg
		return ((90 - getYaw()) % 360);
	}
	
    public void initDefaultCommand() {
        setDefaultCommand(new TankDrive());
    }   
    
	public void resetXY() {
		m_currentX = 0;
		m_currentY = 0;
	}
	
    public void calcXY() {
    	// calculate x,y coordinates when moving in straight lines and turning in place, DOES NOT WORK WITH CURVES (yet)
    	double m_currentDistance = (getRightMasterPosition() + getLeftMasterPosition())/2;
    	double m_distanceTraveled = (m_currentDistance - m_previousDistance);
    	double angle = getAngle();
  
//    	If positive y axis is 0 deg
//    	m_currentX = m_currentX + m_distanceTraveled * Math.sin(Math.toRadians(angle));
//    	m_currentY = m_currentY + m_distanceTraveled * Math.cos(Math.toRadians(angle));
    	
//    	if positive x axis is 0 deg
    	m_currentX = m_currentX + m_distanceTraveled * Math.cos(Math.toRadians(angle));
    	m_currentY = m_currentY + m_distanceTraveled * Math.sin(Math.toRadians(angle));

    	m_previousDistance = m_currentDistance;
    }
    
    public double getXpos() {
    	return m_currentX;
    }
    
    public double getYpos() {
    	return m_currentY;
    }
    
    
    public void reportToSmartDashboard() {
    	SmartDashboard.putNumber("Left Master Voltage", getLeftOutputVoltage());
    	SmartDashboard.putNumber("Right Master Voltage", getRightOutputVoltage());
    	
    	SmartDashboard.putNumber("Left Master Position", getLeftMasterPosition());
    	SmartDashboard.putNumber("Right Master Position", getRightMasterPosition());
    	
    }
    
}

