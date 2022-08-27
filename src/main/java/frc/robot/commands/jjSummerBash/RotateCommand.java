// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.jjSummerBash;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RotateCommand extends CommandBase {
  /** Creates a new RotateCommand. */
  private DrivetrainSubsystem drivetrain;
  private Joystick jstick;
  public RotateCommand(DrivetrainSubsystem drivetrainSubsystem, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
    drivetrain = drivetrainSubsystem;
    jstick = joystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedMult = (-this.jstick.getRawAxis(3)) / 2 + 0.5;
    drivetrain.tankDrive(jstick.getRawAxis(0)*0.3*speedMult, -jstick.getRawAxis(0)*0.3*speedMult);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
