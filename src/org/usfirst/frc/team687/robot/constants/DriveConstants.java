package org.usfirst.frc.team687.robot.constants;

import org.usfirst.frc.team687.robot.utilities.BezierCurve;

public class DriveConstants {
//	TalonSRX Constants
	public static final double kLeftP = 0;
	public static final double kLeftI = 0;
	public static final double kLeftD = 0;
	public static final double kLeftF = 0;
	
	
	public static final double kRightP = 0;
	public static final double kRightI = 0;
	public static final double kRightD = 0;
	public static final double kRightF = 0;
	
	public static final int kLeftAcceleration = 0;
	public static final int kLeftCruiseVelocity = 0;
	
	public static final int kRightAcceleration = 0;
	public static final int kRightCruiseVelocity = 0;

//	Rot PID Constants
	public static final double kRotP = 0;
	public static final double kDriveRotationTolerance = 0;
	public static final double kMaxRotPower = 0.8;
	public static final double kMinRotPower = 0;
	public static final double kRotationalEncoderTolerance = 200;
	
//	Drive Constants, for driving straight
	public static final double kDriveP = 0;
	public static final double kDriveTolerance = 200;
	
//	Physical Robot Constants
	public static double kWheelDiameter = 6;
	public static double kDrivetrainWidth = 25;
	
//	Other Constants 
	public static double kJoystickDeadband = 0.1;
	
	public static double kMinDistToBezierPoint = 100; //distance to target point at bezier curve where robot changes target to nect point
//	public static BezierCurve kTestCurveStraight = new BezierCurve()
	public static double kMaximumTurnRadius = 50;
	
	public static double kRotD = 0;
	public static double kRotMinPower = 0;
	public static double kRotPMaxPower = 1;
	
	public static double kLeftAdjustment = 1;
	public static double kRightAdjustment = 1;

	
	
}
