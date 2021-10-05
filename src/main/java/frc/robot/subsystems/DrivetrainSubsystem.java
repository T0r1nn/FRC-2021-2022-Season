// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

private WPI_TalonFX backLeft = new WPI_TalonFX(0);
private WPI_TalonFX backRight = new WPI_TalonFX(1);
private WPI_TalonFX frontLeft = new WPI_TalonFX(2);
private WPI_TalonFX frontRight = new WPI_TalonFX(3);

SpeedControllerGroup leftMotorGroup = new SpeedControllerGroup(frontLeft, backLeft);
SpeedControllerGroup rightMotorGroup = new SpeedControllerGroup(frontRight, backRight);

private DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {}

  public void arcadeDrive(double forwardVelocity, double rotationalVelocity) {
    differentialDrive.arcadeDrive(forwardVelocity, rotationalVelocity);
  }
}
