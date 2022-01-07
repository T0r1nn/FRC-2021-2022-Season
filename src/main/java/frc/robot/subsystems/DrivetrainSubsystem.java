// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

  WPI_TalonFX rightFront = new WPI_TalonFX(0);
  WPI_TalonFX leftBack = new WPI_TalonFX(1);
  WPI_TalonFX leftFront = new WPI_TalonFX(2);
  WPI_TalonFX rightBack = new WPI_TalonFX(3);

  SpeedControllerGroup right = new SpeedControllerGroup(rightFront, rightBack);

  SpeedControllerGroup left = new SpeedControllerGroup(leftFront, leftBack);

  DifferentialDrive drivetrain = new DifferentialDrive(left, right);

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }
}