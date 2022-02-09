// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

  TalonFX rightFront = new TalonFX(0);
  TalonFX leftBack = new TalonFX(3);
  TalonFX leftFront = new TalonFX(2);
  TalonFX rightBack = new TalonFX(1);

  double inchesPerTick = 0.0092084867;

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    rightBack.follow(rightFront);
    leftBack.follow(leftFront);
    rightFront.setInverted(true);
    rightBack.setInverted(true);
    rightFront.setSensorPhase(false);
    leftFront.setSensorPhase(false);
    rightBack.setSensorPhase(false);
    leftBack.setSensorPhase(false);
    rightFront.setSelectedSensorPosition(0);
    leftFront.setSelectedSensorPosition(0);
    rightBack.setSelectedSensorPosition(0);
    leftBack.setSelectedSensorPosition(0);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);
    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    rightFront.set(TalonFXControlMode.PercentOutput, rightSpeed);
    leftFront.set(TalonFXControlMode.PercentOutput, leftSpeed);
  }

  public void arcadeDrive(double speed, double turn) {
    double xSpeed = MathUtil.clamp(speed, -1.0, 1.0);
    double zRotation = MathUtil.clamp(turn, -1.0, 1.0);

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
    zRotation = Math.copySign(zRotation * zRotation, zRotation);

    double leftSpeed;
    double rightSpeed;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0) {
        leftSpeed = maxInput;
        rightSpeed = xSpeed - zRotation;
      } else {
        leftSpeed = xSpeed + zRotation;
        rightSpeed = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0) {
        leftSpeed = xSpeed + zRotation;
        rightSpeed = maxInput;
      } else {
        leftSpeed = maxInput;
        rightSpeed = xSpeed - zRotation;
      }
    }

    // Normalize the wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
    if (maxMagnitude > 1.0) {
      leftSpeed /= maxMagnitude;
      rightSpeed /= maxMagnitude;
    }

    rightFront.set(TalonFXControlMode.PercentOutput, rightSpeed);
    leftFront.set(TalonFXControlMode.PercentOutput, rightSpeed);
  }

  public double getLeftDistanceTicks() {
    return leftFront.getSelectedSensorPosition();
  }

  public double getRightDistanceTicks() {
    return rightFront.getSelectedSensorPosition();
  }

  public double getRightDistanceInch() {
    return getRightDistanceTicks() * inchesPerTick;
  }

  public double getLeftDistanceInch() {
    return getLeftDistanceTicks() * inchesPerTick;
  }
}