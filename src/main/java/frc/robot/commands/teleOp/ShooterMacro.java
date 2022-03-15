// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOp;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterMacro extends CommandBase {
  /** Creates a new ShooterMacro. */
  private ShooterSubsystem shooter;
  private ConveyorSubsystem conveyor;
  private int time = 0;
  public ShooterMacro(ShooterSubsystem shooter, ConveyorSubsystem conveyor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.conveyor = conveyor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.time++;
    if(this.time < 20){
      shooter.runShooter(-0.7);
    }else{

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time > 40;
  }
}
