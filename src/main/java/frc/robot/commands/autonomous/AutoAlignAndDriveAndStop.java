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
    double Kp = -0.015f;
    double min_command = 0.02f;
    double max_speed = 0.35;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-fphil");
    double tx = table.getEntry("tx").getDouble(0.0);

    double left_command = 0.0;
    double right_command = 0.0;
    double steerWeight = 0.7;

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
    left_command += 1-steerWeight;
    right_command += 1-steerWeight;

    SmartDashboard.putNumber("tx",tx);
    SmartDashboard.putNumber("turnSpeed", Math.copySign(Math.min(Math.abs(left_command),max_speed),left_command/Math.abs(left_command)));

    if(Math.sqrt(Math.pow(odometry.x_position,2)+Math.pow(odometry.y_position,2)) > 84){
      drivetrainSubsystem.tankDrive(0, 0);
    }else{
      drivetrainSubsystem.tankDrive(Math.copySign(Math.min(Math.abs(left_command),max_speed),left_command), Math.copySign(Math.min(Math.abs(right_command),max_speed),right_command));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.sqrt(Math.pow(odometry.x_position,2)+Math.pow(odometry.y_position,2)) > 84;
  }
}
