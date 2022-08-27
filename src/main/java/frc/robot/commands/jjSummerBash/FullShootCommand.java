// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.jjSummerBash;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FullShootCommand extends CommandBase {
  /** Creates a new ShootCommand. */
  IntakeSubsystem intake;
  ConveyorSubsystem conveyor;
  ShooterSubsystem shooter;
  public FullShootCommand(IntakeSubsystem intakeSubsystem, ConveyorSubsystem conveyorSubsystem, ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = intakeSubsystem;
    conveyor = conveyorSubsystem;
    shooter = shooterSubsystem;
    addRequirements(intakeSubsystem, conveyorSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conveyor.runConveyor(-0.75);
    shooter.runShooter(-0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.runConveyor(0);
    shooter.runShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
