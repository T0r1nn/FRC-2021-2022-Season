// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOp;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {

  private ShooterSubsystem subsystem;
  private Joystick buttonBoard;
  double shooterSpeed = 0.0;
  double shooterMult = 1.0;

  /** Creates a new DriveCommand. */
  public ShooterCommand(ShooterSubsystem subsystemParam, Joystick buttonBoardParam) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystemParam;
    this.buttonBoard = buttonBoardParam;

    addRequirements(this.subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(this.buttonBoard.getRawAxis(0) < -0.5){
      shooterMult += 0.001;
    }else if(this.buttonBoard.getRawAxis(1) > 0.5){
      shooterMult -= 0.001;
    }
    if(this.buttonBoard.getRawButton(2)){
      shooterSpeed = shooterMult;
    }else{
      shooterSpeed = 0;
    }
    subsystem.runShooter(shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.runShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
