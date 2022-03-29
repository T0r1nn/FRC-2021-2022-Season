// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOp;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WinchSubsystem;

public class SnapWinchCommand extends CommandBase {
  /** Creates a new SnapWinchCommand. */
  WinchSubsystem winch;
  double upPos = 0;
  double downPos = 1;
  boolean toggled = false;

  public SnapWinchCommand(WinchSubsystem sub) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sub);
    winch = sub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    toggled = !toggled;
    if(toggled){
      winch.driveTo(downPos);
    }else{
      winch.driveTo(upPos);
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
