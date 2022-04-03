// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOp;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SecondaryClimberSystem;

public class SetupClimbers extends CommandBase {
  /** Creates a new SetupClimbers. */
  private ClimberSubsystem climberSubsystem;
  private SecondaryClimberSystem secondaryClimberSystem;
  private boolean done = false;
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
    int doneCount = 0;
    SmartDashboard.putNumber("climber 1 pos",this.climberSubsystem.getClimberLeft().getEncoder().getPosition());
    SmartDashboard.putNumber("climber 2 pos",this.secondaryClimberSystem.getClimberLeft().getEncoder().getPosition());
    if(climberSubsystem.getClimberLeft().getEncoder().getPosition() > -142){
      climberSubsystem.runClimber(-0.8);
    }else{
      climberSubsystem.runClimber(0);
      doneCount++;
    }
    if(secondaryClimberSystem.getClimberLeft().getEncoder().getPosition() < 69){
      secondaryClimberSystem.runClimber(0.8);
    }else{
      secondaryClimberSystem.runClimber(0);
      doneCount++;
    }
    done = doneCount == 2;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
