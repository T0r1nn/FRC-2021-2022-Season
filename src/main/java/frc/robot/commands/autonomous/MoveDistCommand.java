// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.misc.OdometryCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class MoveDistCommand extends CommandBase {
  /** Creates a new MoveDistCommand. */
  double targetDistance = 0;
  double moveSpeed = 0;
  double startingX = 0;
  double startingY = 0;
  OdometryCommand odometryCommand;
  DrivetrainSubsystem drivetrainSubsystem;

  public MoveDistCommand(double dist, double speed, OdometryCommand odometry, DrivetrainSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    targetDistance = dist;
    moveSpeed = speed;
    odometryCommand = odometry;
    drivetrainSubsystem = drivetrain;
    startingX = odometry.x_position;
    startingY = odometry.y_position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainSubsystem.tankDrive(moveSpeed, moveSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.sqrt(Math.pow(odometryCommand.x_position - startingX, 2)
        + Math.pow(odometryCommand.y_position - startingY, 2)) >= targetDistance;
  }
}
