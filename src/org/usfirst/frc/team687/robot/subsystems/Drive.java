package org.usfirst.frc.team687.robot.subsystems;

import org.usfirst.frc.team687.robot.RobotMap;
import org.usfirst.frc.team687.robot.commands.drive.teleop.ArcadeDrive;
import org.usfirst.frc.team687.robot.commands.drive.teleop.TankDrive;
import org.usfirst.frc.team687.robot.constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Drive extends Subsystem {

	private final TalonSRX m_leftMaster, m_leftSlave1, m_leftSlave2;
	private final TalonSRX m_rightMaster, m_rightSlave1, m_rightSlave2;
	private final AHRS m_nav;
	
	private double m_previousDistance;
    private double m_currentX, m_currentY;
    
	public Drive() {
		
		m_nav = new AHRS(SPI.Port.kMXP);
		
		m_leftMaster = new TalonSRX(RobotMap.kLeftMasterTalonSRXID);
		m_leftSlave1 = new TalonSRX(RobotMap.kLeftSlaveTalonSRX1ID);
		m_leftSlave2 = new TalonSRX(RobotMap.kLeftSlaveTalonSRX2ID);
		
		m_rightMaster = new TalonSRX(RobotMap.kRightMasterTalonSRXID);
		m_rightSlave1 = new TalonSRX(RobotMap.kRightSlaveTalonSRX1ID);
		m_rightSlave2 = new TalonSRX(RobotMap.kRightSlaveTalonSRX2ID);
		
		
		m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		
		m_rightSlave1.follow(m_rightMaster);
		m_rightSlave2.follow(m_rightMaster);
		
		m_leftSlave1.follow(m_leftMaster);
		m_leftSlave2.follow(m_leftMaster);
				
		m_leftMaster.setInverted(true);
		m_leftSlave1.setInverted(true);
		m_leftSlave2.setInverted(true);
		
		m_rightMaster.setInverted(true);
		m_rightSlave1.setInverted(true);
		m_rightSlave2.setInverted(true)
		;
		m_leftMaster.setSensorPhase(false);
		m_rightMaster.setSensorPhase(false);
		
		m_rightMaster.config_kP(0, DriveConstants.kRightP, 0);
		m_rightMaster.config_kI(0, DriveConstants.kRightI, 0);
		m_rightMaster.config_kD(0, DriveConstants.kRightD, 0);
		m_rightMaster.config_kF(0, DriveConstants.kRightF, 0);
		
		m_leftMaster.config_kP(0, DriveConstants.kLeftP, 0);//CHANGE THESE TO RIGHT
		m_leftMaster.config_kI(0, DriveConstants.kLeftI, 0);
		m_leftMaster.config_kD(0, DriveConstants.kLeftD, 0);
		m_leftMaster.config_kF(0, DriveConstants.kLeftF, 0);
		
		m_leftMaster.configMotionAcceleration(DriveConstants.kLeftAcceleration, 0);
		m_leftMaster.configMotionCruiseVelocity(DriveConstants.kLeftCruiseVelocity, 0);
		
		m_rightMaster.configMotionAcceleration(DriveConstants.kRightAcceleration, 0);
		m_rightMaster.configMotionCruiseVelocity(DriveConstants.kRightCruiseVelocity, 0);
		
		m_leftMaster.setNeutralMode(NeutralMode.Brake);
		m_leftSlave1.setNeutralMode(NeutralMode.Brake);
		m_leftSlave2.setNeutralMode(NeutralMode.Brake);
		
		m_rightMaster.setNeutralMode(NeutralMode.Brake);
		m_rightSlave1.setNeutralMode(NeutralMode.Brake);
		m_rightSlave2.setNeutralMode(NeutralMode.Brake);
		
	}
	
	public void setPower(double leftPower, double rightPower) {

		m_leftMaster.set(ControlMode.PercentOutput, leftPower);
		m_rightMaster.set(ControlMode.PercentOutput, rightPower);
    }
	
	public void setPowerZero() {
		m_leftMaster.set(ControlMode.PercentOutput, 0);
		m_rightMaster.set(ControlMode.PercentOutput, 0);
	}
	
	public void setPositionMotionMagic(double leftPosition, double rightPosition) {
		m_leftMaster.set(ControlMode.MotionMagic, leftPosition);
		m_rightMaster.set(ControlMode.MotionMagic, rightPosition);
	}
	
	public void resetEncoders() {
		m_leftMaster.setSelectedSensorPosition(0, 0, 0);
		m_rightMaster.setSelectedSensorPosition(0, 0, 0);
	}
	public double getLeftOutputVoltage() {
		return m_leftMaster.getMotorOutputVoltage();
	}
	
	public double getLeftMasterCurrent() {
		return m_leftMaster.getOutputCurrent();
	}
	
	public double getLeftMasterPosition() {
		return m_leftMaster.getSelectedSensorPosition(0);
	}
	
	public double getLeftMasterSpeed() {
		return m_leftMaster.getSelectedSensorVelocity(0);
	}
	
	
	public double getRightOutputVoltage() {
		return m_rightMaster.getMotorOutputVoltage();
	}
	
	public double getRightMasterCurrent() {
		return m_rightMaster.getOutputCurrent();
	}
	
	public double getRightMasterPosition() {
		return m_rightMaster.getSelectedSensorPosition(0);
	}
	
	public double getRightMasterSpeed() {
		return m_rightMaster.getSelectedSensorVelocity(0);
	}
	
	
	public double getRawYaw() {
		return m_nav.getYaw();
	}
	
	public void resetYaw() {
		m_nav.reset();
	}
	
	public double getAverageEncoderPosition() {
		return (getRightMasterPosition() + getLeftMasterPosition())/2;
	}
	
	public double getAngle() {
//		converts angle from -180 to 180 to 0 to 360	
//		sets positive y as 0 deg, robot's front is 0 deg
		return (360 - getRawYaw()) % 360;
		
	}
	
    public void initDefaultCommand() {
        setDefaultCommand(new ArcadeDrive());
    }   
    
	public void resetXY() {
		m_currentX = 0;
		m_currentY = 0;
	}
	
    public void calcXY() {
    	// calculate x,y coordinates when moving in straight lines and turning in place, DOES NOT WORK
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
    	
    	SmartDashboard.putNumber("Yaw", getRawYaw());
    	
    }
    
}

