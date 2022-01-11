// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.autonomous.MoveDistCommand;
import frc.robot.commands.misc.OdometryCommand;
import frc.robot.commands.misc.WaitUntilTimeCommand;
import frc.robot.commands.teleOp.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public OdometryCommand odometry;
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final Joystick leftJoystick = new Joystick(0);
  private final Joystick rightJoystick = new Joystick(1);
  private final DriveCommand driveCommand = new DriveCommand(drivetrainSubsystem, leftJoystick, rightJoystick);
  private final MoveDistCommand autonomousMove = new MoveDistCommand(24, 0.35, odometry, drivetrainSubsystem);
  private final WaitUntilTimeCommand autonomousWait = new WaitUntilTimeCommand(5);
  private Command autonomous;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    autonomous = new SequentialCommandGroup(autonomousMove, autonomousWait);
    drivetrainSubsystem.setDefaultCommand(driveCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  public Command getAutoCommand() {
    return autonomous;
  }
}
