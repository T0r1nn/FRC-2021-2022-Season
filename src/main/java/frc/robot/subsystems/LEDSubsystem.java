// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private Spark blinkin1 = new Spark(2);
  private Spark blinkin2 = new Spark(3);
  private double blinkin1Pattern = 0.0;
  private double blinkin2Pattern = 0.0;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    blinkin1.set(blinkin1Pattern);
    blinkin2.set(blinkin2Pattern);
  }

  public void setBlinkin1Pattern(double blinkin1Pattern) {
      this.blinkin1Pattern = blinkin1Pattern;
  }

  public void setBlinkin2Pattern(double blinkin2Pattern) {
      this.blinkin2Pattern = blinkin2Pattern;
  }
}
