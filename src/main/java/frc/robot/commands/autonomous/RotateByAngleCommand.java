// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RotateByAngleCommand extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  double angle = 0;
  double delta = 2;
  /** Creates a new AutoAlignCommand. */
  public RotateByAngleCommand(DrivetrainSubsystem subsystem, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrainSubsystem = subsystem;
    addRequirements(subsystem);
    this.angle = (angle+Math.toDegrees(Constants.odometry.rotation))%360;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double heading_error = (angle-Math.toDegrees(Constants.odometry.rotation));
    delta = heading_error;
    double steering_adjust = 0;
    double Kp = -0.015;
    double min_command = 0.2;
    double left_command = 0;
    double right_command = 0;
    double max_speed = 0.3;

    if (delta > 1.0)
    {
      steering_adjust = Kp*heading_error - min_command;
    }
    else if (delta < -1.0)
    {
      steering_adjust = Kp*heading_error + min_command;
    }
    left_command += steering_adjust;
    right_command -= steering_adjust;
    SmartDashboard.putString("state","ROTATETOANGLE");

    drivetrainSubsystem.tankDrive(max_speed*Math.copySign(Math.min(Math.abs(left_command),1),left_command), max_speed*Math.copySign(Math.min(Math.abs(right_command),1),right_command));
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
