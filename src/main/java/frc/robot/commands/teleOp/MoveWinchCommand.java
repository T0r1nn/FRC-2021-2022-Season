// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOp;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WinchSubsystem;

public class MoveWinchCommand extends CommandBase {
  /** Creates a new MoveWinchCommand. */
  WinchSubsystem winch;
  Joystick leftJoystick;
  Joystick rightJoystick;
  Joystick buttons;
  public MoveWinchCommand(WinchSubsystem winchSubsystem, Joystick lefJoystick, Joystick righJoystick, Joystick buttonBoard) {
    // Use addRequirements() here to declare subsystem dependencies.
    winch = winchSubsystem;
    rightJoystick = righJoystick;
    leftJoystick = lefJoystick;
    buttons = buttonBoard;
    addRequirements(winchSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(leftJoystick.getRawButton(1)){
      winch.driveWinch(0.4);
    }else if(rightJoystick.getRawButton(1) || buttons.getRawButton(2)){
      winch.driveWinch(-0.4);
    }else if(leftJoystick.getRawButton(2)){
      winch.driveWinch(0.2);
    }else if(rightJoystick.getRawButton(2)){
      winch.driveWinch(-0.2);
    }else{
      winch.driveWinch(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    winch.driveWinch(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
