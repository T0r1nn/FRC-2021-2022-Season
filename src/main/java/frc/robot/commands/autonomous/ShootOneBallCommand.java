// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootOneBallCommand extends CommandBase {
  ShooterSubsystem shooterSubsystem;
  ConveyorSubsystem conveyorSubsystem;
  int ticksLeft = 20;

  /** Creates a new ShootOneBallCommand. */
  public ShootOneBallCommand(ShooterSubsystem shooter, ConveyorSubsystem conveyor) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter,conveyor);
    shooterSubsystem = shooter;
    conveyorSubsystem = conveyor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.runShooter(0.7);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.runShooter(-0.7);
    if(ticksLeft <= 0){
      conveyorSubsystem.runConveyor(-0.75);
    }
    ticksLeft--;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.runShooter(0);
    conveyorSubsystem.runConveyor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ticksLeft < -20;
  }
}
