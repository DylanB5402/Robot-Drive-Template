package org.usfirst.frc.team687.robot.commands.drive.teleop;

import org.usfirst.frc.team687.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Field Relative Drive:
 * Based on standard 2 stick arcade, 1 stick control's robot's turning relative to the field/navx, other controls straight power
 */
public class FieldRelativeDrive extends Command {

    public FieldRelativeDrive() {
        requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
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
