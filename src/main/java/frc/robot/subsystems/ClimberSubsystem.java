// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase {
  CANSparkMax climberLeft = new CANSparkMax(7,MotorType.kBrushless);
  CANSparkMax climberRight = new CANSparkMax(8,MotorType.kBrushless);
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runClimber(double speed){
    climberLeft.set(speed);
    climberRight.set(speed);
  }
}
