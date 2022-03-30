// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RotateToAngleCommand extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  double angle = 0;
  double delta = 0;
  /** Creates a new AutoAlignCommand. */
  public RotateToAngleCommand(DrivetrainSubsystem subsystem, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrainSubsystem = subsystem;
    addRequirements(subsystem);
    this.angle = angle%360;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curtheta = Constants.odometry.rotation % (Math.PI*2);
    curtheta *= 180/Math.PI;
    double leftRotDist = angle-curtheta;
    double rightRotDist = 360-leftRotDist;
    delta = 0;
    if(leftRotDist < rightRotDist){
      delta = leftRotDist;
    }else{
      delta = -rightRotDist;
    }

    double heading_error = -delta;
    double steering_adjust = 0;
    double Kp = -0.015;
    double min_command = 0.1;
    double left_command = 0;
    double right_command = 0;
    double max_speed = 0.6;

    if (delta > 1.0)
    {
      steering_adjust = Kp*heading_error - min_command;
    }
    else if (delta < 1.0)
    {
      steering_adjust = Kp*heading_error + min_command;
    }
    left_command += steering_adjust;
    right_command -= steering_adjust;

    drivetrainSubsystem.tankDrive(Math.copySign(Math.min(Math.abs(left_command),max_speed),left_command), Math.copySign(Math.min(Math.abs(right_command),max_speed),right_command));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(delta) < 1;
  }
}
