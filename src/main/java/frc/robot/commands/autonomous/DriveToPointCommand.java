// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToPointCommand extends CommandBase {
  private double targetX = 0;
  private double targetY = 0;
  private double tolerance = 2;
  private DrivetrainSubsystem drivetrain;
  /** Creates a new DriveToPointCommand. */
  public DriveToPointCommand(DrivetrainSubsystem drSubsystem, double tx, double ty, double tolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    //Sets target position and how far off of the position the robot tries to be.
    targetX = tx;
    targetY = ty;
    this.tolerance = tolerance;
    drivetrain = drSubsystem;
    addRequirements(drSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  //Gets how fast the robot should drive at a certain distance from the target.
  public double getDriveSpeed(double dist){
    return -1/(0.1*dist + 1.5) + 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Kp = -0.015f;
    double min_command = 0.02f;
    double max_speed = 1;
    double speed = 0.4;

    double tx = 90-Math.toDegrees(Math.atan2(targetY-Constants.odometry.y_position,targetX-Constants.odometry.x_position));
    tx = (Math.toDegrees(Constants.odometry.rotation))%360 - tx;
    
    while(tx < 0){
      tx+=360;
    }
    while(tx > 360){
      tx-=360;
    }
    if(tx > 180){
      tx = 360 - tx;
    }else{
      tx = -tx;
    }

    double left_command = 0.0;
    double right_command = 0.0;
    double steerWeight = 0.4;

    double heading_error = -tx;
    double steering_adjust = 0.0f;
    if (tx > 1.0)
    {
      steering_adjust = Kp*heading_error - min_command;
    }
    else if (tx < 1.0)
    {
      steering_adjust = Kp*heading_error + min_command;
    }
    left_command += steering_adjust;
    right_command -= steering_adjust;
    left_command *= steerWeight;
    right_command *= steerWeight;
    left_command += (1-steerWeight);
    right_command += (1-steerWeight);
    SmartDashboard.putString("state","DRIVETOPOINT");

    drivetrain.tankDrive(speed * Math.copySign(Math.min(Math.abs(left_command),max_speed),left_command), speed * Math.copySign(Math.min(Math.abs(right_command),max_speed),right_command));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Constants.odometry.distTo(targetX,targetY) < tolerance;
  }
}
