package org.usfirst.frc.team687.robot.subsystems;

import org.usfirst.frc.team687.robot.RobotMap;
import org.usfirst.frc.team687.robot.constants.ClawConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Claw extends Subsystem {

	private final TalonSRX m_artic;
	
    public Claw() {
	   
    	m_artic = new TalonSRX(RobotMap.kArticTalonSRXID);
    	m_artic.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    	m_artic.config_kP(0, ClawConstants.kClawP, 0);
    	m_artic.config_kI(0, ClawConstants.kClawI, 0);
    	m_artic.config_kD(0, ClawConstants.kClawD, 0);
    	m_artic.config_kF(0, ClawConstants.kClawF, 0);    	
    }
    
    public void setPower(double power) {
    	m_artic.set(ControlMode.PercentOutput, power);
    }
    
    public void setPosition(double position) {
    	m_artic.set(ControlMode.Position, position);
    }
    
    public void resetEncoders() {
    	m_artic.setSelectedSensorPosition(0, 0, 0);
    }
    
    public double getPosition() {
    	return m_artic.getSelectedSensorPosition(0);
    }
    
   public double getVoltage() {
	   return m_artic.getMotorOutputVoltage();
   }
   
   public double getCurrent() {
	   return m_artic.getOutputCurrent();
   }
   
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

