// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOp;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.misc.OdometryCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveCommand extends CommandBase {

  private DrivetrainSubsystem subsystem;
  private Joystick leftJoystick;
  OdometryCommand odometry = new OdometryCommand();

  private Joystick rightJoystick;

  /** Creates a new DriveCommand. */
  public DriveCommand(DrivetrainSubsystem subsystemParam, Joystick leftJoystickParam, Joystick rightJoystickParam) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystemParam;
    this.leftJoystick = leftJoystickParam;
    this.rightJoystick = rightJoystickParam;

    addRequirements(this.subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speedMult = 0.0;
    speedMult = (-this.leftJoystick.getRawAxis(3)) / 4 + 0.25;

    subsystem.tankDrive(-this.leftJoystick.getRawAxis(1) * speedMult, -this.rightJoystick.getRawAxis(1) * speedMult);

    odometry.execute(subsystem);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
