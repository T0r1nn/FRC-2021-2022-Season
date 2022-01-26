// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.misc;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class IdleCommand extends CommandBase {
  DrivetrainSubsystem drivetrain;

  /** Creates a new IdleCommand. */
  public IdleCommand(DrivetrainSubsystem drivetrainArgument) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainArgument);
    drivetrain = drivetrainArgument;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
