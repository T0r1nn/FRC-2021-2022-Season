// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOp;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends CommandBase {
  private ClimberSubsystem subsystem;
  private Joystick buttonBoard;
  private ADXRS450_Gyro balanceGyro;
  /** Creates a new ClimberCommand. */
  public ClimberCommand(ClimberSubsystem subsystemParam, ADXRS450_Gyro balanceGyro, Joystick buttonBoardParam) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystemParam;
    this.buttonBoard = buttonBoardParam;
    this.balanceGyro = balanceGyro;
    addRequirements(this.subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("angle",balanceGyro.getAngle());
    double climberSpeed = 0.0;
    if(this.buttonBoard.getRawButton(6)){
      climberSpeed = 0.2;
    }else if(this.buttonBoard.getRawButton(3)){
      climberSpeed = -0.2;
    }else{
      climberSpeed = 0.0;
    }
    subsystem.runClimber(climberSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.runClimber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
