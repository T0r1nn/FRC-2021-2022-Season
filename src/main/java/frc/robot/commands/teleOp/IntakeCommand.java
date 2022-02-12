// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOp;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

  private IntakeSubsystem subsystem;
  private Joystick buttonBoard;

  /** Creates a new DriveCommand. */
  public IntakeCommand(IntakeSubsystem subsystemParam, Joystick buttonBoardParam) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystemParam;
    this.buttonBoard = buttonBoardParam;

    addRequirements(this.subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double intakeSpeed = 0.0;
    if(this.buttonBoard.getRawButton(11)){
      intakeSpeed = 0.3;
    }else if(this.buttonBoard.getRawButton(10)){
      intakeSpeed = -0.3;
    }else{
      intakeSpeed = 0.0;
    }
    subsystem.runIntake(intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
