// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOp;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {

  private ShooterSubsystem subsystem;
  private Joystick buttonBoard;
  double shooterSpeed = -0.5;
  boolean shootButtonPressed = false;
  boolean shooterToggled = false;

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
    if(this.buttonBoard.getRawAxis(0) < -0.75){
      shooterSpeed += 0.001;
    }else if(this.buttonBoard.getRawAxis(1) < -0.75){
      shooterSpeed -= 0.001;
    }
    if(this.buttonBoard.getRawButton(2) && !shootButtonPressed){
      shootButtonPressed = true;
      shooterToggled = !shooterToggled;
    }else if(!this.buttonBoard.getRawButton(2)){
      shootButtonPressed = false;
    }
    SmartDashboard.putNumber("Shooter Speed",-100*shooterSpeed);
    subsystem.runShooter(shooterToggled?shooterSpeed:0);
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
