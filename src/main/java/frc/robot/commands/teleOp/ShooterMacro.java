// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOp;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterMacro extends CommandBase {
  /** Creates a new ShooterMacro. */
  private ShooterSubsystem shooter;
  private ConveyorSubsystem conveyor;
  private int time = 100;
  private double[] distances = new double[]{};
  private double[] speeds = new double[]{};

  public ShooterMacro(ShooterSubsystem shooter, ConveyorSubsystem conveyor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.conveyor = conveyor;
    addRequirements(shooter,conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.time = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.time++;
    shooter.runShooter(Constants.shooterSpeed);
    if(this.time > 20){
      conveyor.runConveyor(-0.65);
    }
    SmartDashboard.putNumber("Distance to target", calcDistance());
  }

  public double calcDistance(){
    double reflectiveTapeHeight = 102.8125;
    double limelightHeight = 0;
    double limelightAngle = 0;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-shoot");
    double tx = table.getEntry("ty").getDouble(0.0);
    double totalAngle = tx+limelightAngle;
    return (reflectiveTapeHeight-limelightHeight)/Math.tan(Math.toRadians(totalAngle));
  }

  public double calcSpeed(){
    double dist = calcDistance();
    if(dist < distances[0] || dist > distances[distances.length-1]){
      return 0.0;
    }
    double sp = 0;
    for(int i = 0; i < distances.length-1; i++){
      if(dist > distances[i] && dist < distances[i+1]){
        double percentage = (dist-distances[i])/(distances[i+1]-distances[i]);
        sp = (percentage*(speeds[i+1]-speeds[i]) + speeds[i]);
      }
    }
    return sp;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.runShooter(0);
    conveyor.runConveyor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time > 75;
  }
}
