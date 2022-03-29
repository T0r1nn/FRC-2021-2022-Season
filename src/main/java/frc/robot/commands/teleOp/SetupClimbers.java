// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOp;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SecondaryClimberSystem;

public class SetupClimbers extends CommandBase {
  /** Creates a new SetupClimbers. */
  private ClimberSubsystem climberSubsystem;
  private SecondaryClimberSystem secondaryClimberSystem;
  public SetupClimbers(ClimberSubsystem climber, SecondaryClimberSystem secondaryClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber,secondaryClimber);
    climberSubsystem = climber;
    secondaryClimberSystem = secondaryClimber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(climberSubsystem.getClimberLeft().getEncoder().getPosition() > -142){
      climberSubsystem.runClimber(-0.8);
    }else{
      climberSubsystem.runClimber(0);
    }
    if(secondaryClimberSystem.getClimberLeft().getEncoder().getPosition() < 71){
      secondaryClimberSystem.runClimber(0.8);
    }else{
      secondaryClimberSystem.runClimber(0);
    }
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
