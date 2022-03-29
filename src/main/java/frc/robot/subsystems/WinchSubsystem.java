// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WinchSubsystem extends SubsystemBase {
  /** Creates a new WinchSubsystem. */
  Spark winch = new Spark(4);
  boolean targeting = false;
  double target = 0;

  public WinchSubsystem() {}

  public void driveWinch(double speed){
    winch.set(speed);
  }

  public double getPosition(){
    return winch.get();
  }

  public void driveTo(double t){
    targeting = true;
    target = t;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(targeting){
      if(Math.abs(winch.get()-target) < 1){
        targeting = false;
      }else if(target > winch.get()){
        winch.set(0.5);
      }else if(target < winch.get()){
        winch.set(-0.5);
      }
    }
  }
}
