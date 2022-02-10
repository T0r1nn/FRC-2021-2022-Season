// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class ClimberSubsystem extends SubsystemBase {
  PWMSparkMax climberLeft = new PWMSparkMax(2);
  PWMSparkMax climberRight = new PWMSparkMax(3);
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
