/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

 // Mahesh new Architecture fix - 1/13/2023  -- 

//KEEP KEEP KEEP
public class DriveJoystick extends CommandBase {

  public DriveJoystick() {
    //requires(Robot.driveBase); // there is really no need for this requires in previous version - Mahesh 1/13/2023 - commenting - will see how this works seems to be old command framework code
    addRequirements(Robot.driveBase);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    Robot.driveBase.drive(Robot.oi.getDriveJoystick());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  // @Override
  // public void end(boolean interrupted ) {
  // }

  // // Called when another command which requires one or more of the same
  // // subsystems is scheduled to run
  // @Override
  // protected void interrupted() {
  //   end();
  // }
}
