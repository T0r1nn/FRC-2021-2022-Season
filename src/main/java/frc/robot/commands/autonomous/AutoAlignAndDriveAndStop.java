// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.misc.OdometryCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoAlignAndDriveAndStop extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private OdometryCommand odometry;
  /** Creates a new AutoSeekCommand. */
  public AutoAlignAndDriveAndStop(DrivetrainSubsystem subsystem, OdometryCommand odometry) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.odometry = odometry;
    drivetrainSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //P value of p loop
    double Kp = -0.015f;
    //Minimum speed required to move the robot
    double min_command = 0.02f;
    //Maximum speed robot is allowed to move
    double max_speed = 0.35;

    //Accessing limelight and finding the error in x angle 
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-fphil");
    double tx = table.getEntry("tx").getDouble(0.0);

    //Left and right wheel speeds
    double left_command = 0.0;
    double right_command = 0.0;
    //How much of the total speed is used for steering, the rest is used for moving forward
    double steerWeight = 0.7;

    //Error in the robot's heading
    double heading_error = -tx;
    //How much the robot needs to steer to face the target
    double steering_adjust = 0.0f;
    //Calculates steering_adjust
    if (tx > 1.0)
    {
      steering_adjust = Kp*heading_error - min_command;
    }
    else if (tx < 1.0)
    {
      steering_adjust = Kp*heading_error + min_command;
    }
    //Turns steering_adjust into movement
    left_command += steering_adjust;
    right_command -= steering_adjust;
    left_command *= steerWeight;
    right_command *= steerWeight;
    //Adds forward movement
    left_command += 1-steerWeight;
    right_command += 1-steerWeight;

    //Sends debug data to shuffleboard
    SmartDashboard.putNumber("tx",tx);
    SmartDashboard.putNumber("turnSpeed", Math.copySign(Math.min(Math.abs(left_command),max_speed),left_command/Math.abs(left_command)));

    //If you were a certain distance away from where you started, stop
    if(Math.sqrt(Math.pow(odometry.x_position,2)+Math.pow(odometry.y_position,2)) > 84){
      drivetrainSubsystem.tankDrive(0, 0);
    }else{
      //Else, drive straight
      drivetrainSubsystem.tankDrive(Math.copySign(Math.min(Math.abs(left_command),max_speed),left_command), Math.copySign(Math.min(Math.abs(right_command),max_speed),right_command));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Stops the robot
    drivetrainSubsystem.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Finishes when the distance from the start is greater than a certain distance
    return Math.sqrt(Math.pow(odometry.x_position,2)+Math.pow(odometry.y_position,2)) > 84;
  }
}
