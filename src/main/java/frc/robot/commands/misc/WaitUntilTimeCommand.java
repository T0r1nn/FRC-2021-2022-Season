// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.misc;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitUntilTimeCommand extends CommandBase {
  /** Creates a new WaitUntilTimeCommand. */
  double targetTime = 0;
  double startTime = 0;

  public WaitUntilTimeCommand(double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    targetTime = time;
    startTime = Timer.getMatchTime();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetTime = SmartDashboard.getNumber("delay", 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return startTime-Timer.getMatchTime() > targetTime;
  }
}
