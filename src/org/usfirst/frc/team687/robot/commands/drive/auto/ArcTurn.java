package org.usfirst.frc.team687.robot.commands.drive.auto;

import org.usfirst.frc.team687.robot.Robot;
import org.usfirst.frc.team687.robot.constants.DriveConstants;
import org.usfirst.frc.team687.robot.utilities.NerdyMath;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Arc turning
 * @author Ted Lin
 */

public class ArcTurn extends Command {

    private double m_desiredAngle;
    private boolean m_isRightPowered;

    private double m_startTime, m_timeout;
    private double m_error;

    private double m_sign;

    /**
     * Arc Turn
     * 
     * @param desiredAngle
     * @param isRightPowered
     * @param timeout
     * @param sign
     *            (+1.0 or -1.0)
     */
    public ArcTurn(double desiredAngle, boolean isRightPowered, double timeout, double sign) {
	m_desiredAngle = desiredAngle;
	m_isRightPowered = isRightPowered;
	m_timeout = timeout;
	m_sign = Math.signum(sign);

	requires(Robot.drive);
    }

    @Override
    protected void initialize() {
	SmartDashboard.putString("Current Drive Command", "ArcTurn");
	
	m_startTime = Timer.getFPGATimestamp();
    }

    @Override
    protected void execute() {
	double robotAngle = (360 - Robot.drive.getRawYaw()) % 360;
	m_error = -m_desiredAngle - robotAngle;
	m_error = (m_error > 180) ? m_error - 360 : m_error;
	m_error = (m_error < -180) ? m_error + 360 : m_error;
	double rotPower = DriveConstants.kRotP * m_error * 1.3; // multiplied by 2 because the rotational component is
							      // only added to one side of the drivetrain

	double rawSign = Math.signum(rotPower);
	rotPower = NerdyMath.threshold(Math.abs(rotPower), DriveConstants.kRotMinPower, DriveConstants.kRotPMaxPower)
		* rawSign;
	rotPower = Math.abs(rotPower) * m_sign;

	if (m_isRightPowered) {
	    Robot.drive.setPower(0, rotPower);
	} else if (!m_isRightPowered) {
	    Robot.drive.setPower(-rotPower, 0);
	}
    }

    @Override
    protected boolean isFinished() {
	return Math.abs(m_error) < DriveConstants.kDriveRotationTolerance
		|| Timer.getFPGATimestamp() - m_startTime > m_timeout;
    }

    @Override
    protected void end() {
	Robot.drive.setPowerZero();;
    }

    @Override
    protected void interrupted() {
	end();
    }

}