// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.autonomous.ShootOneBallCommand;
import frc.robot.commands.misc.AutoAlignAndDrive;
import frc.robot.commands.misc.AutoAlignCommand;
import frc.robot.commands.misc.IdleCommand;
import frc.robot.commands.misc.OdometryCommand;
import frc.robot.commands.misc.WaitUntilTimeCommand;
import frc.robot.commands.teleOp.DriveCommand;
import frc.robot.commands.teleOp.IntakeCommand;
import frc.robot.commands.teleOp.ClimberCommand;
import frc.robot.commands.teleOp.ConveyorCommand;
import frc.robot.commands.teleOp.ShooterCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  public OdometryCommand odometry = new OdometryCommand();
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final Joystick leftJoystick = new Joystick(0);
  private final Joystick rightJoystick = new Joystick(1);
  public final Joystick buttonBoard = new Joystick(2);
  private ADXRS450_Gyro climberBalancerGyro = new ADXRS450_Gyro();

  private final DriveCommand driveCommand = new DriveCommand(drivetrainSubsystem, leftJoystick, rightJoystick);
  private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem, buttonBoard);
  private final ConveyorCommand conveyorCommand = new ConveyorCommand(conveyorSubsystem, buttonBoard);
  private final ShooterCommand shooterCommand = new ShooterCommand(shooterSubsystem, buttonBoard);
  private final ClimberCommand climberCommand = new ClimberCommand(climberSubsystem, climberBalancerGyro, buttonBoard);
  private final WaitUntilTimeCommand autonomousWait = new WaitUntilTimeCommand(8);
  private final ShootOneBallCommand autonomousShoot = new ShootOneBallCommand(shooterSubsystem, conveyorSubsystem);
  private final IdleCommand idle = new IdleCommand(drivetrainSubsystem);
  public final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-fphil");
  public final NetworkTableEntry tx = table.getEntry("tx");
  public final NetworkTableEntry ty = table.getEntry("ty");
  public final NetworkTableEntry ta = table.getEntry("ta");
  private final JoystickButton autoAlignButton = new JoystickButton(buttonBoard, 5);
  private final JoystickButton autoAlignAndDriveButton = new JoystickButton(buttonBoard, 4);
  private final AutoAlignCommand autoAlignCommand = new AutoAlignCommand(drivetrainSubsystem);
  private final AutoAlignAndDrive autoAlignAndDrive = new AutoAlignAndDrive(drivetrainSubsystem);
  private final AutoAlignAndDrive autonomousDrive = new AutoAlignAndDrive(drivetrainSubsystem);

  private Command teleOp;
  private Command autonomous;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    
    configureButtonBindings();
    autonomous = new SequentialCommandGroup(autonomousShoot,new ParallelRaceGroup(new WaitCommand(SmartDashboard.getNumber("delay", 0.0)), idle), new ParallelRaceGroup(new WaitCommand(2),autonomousDrive));
    teleOp = new ParallelCommandGroup(intakeCommand, climberCommand, conveyorCommand, shooterCommand);
    PortForwarder.add(5800, "photonvision.local", 5800);
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
    autoAlignButton.whenHeld(autoAlignCommand);
    autoAlignAndDriveButton.whenHeld(autoAlignAndDrive);
  }

  public Command getAutoCommand() {
    return autonomous;
  }

  public Command getTeleOpCommand() {
    return teleOp;
  }

  public Command getDriveCommand() {
    return driveCommand;
  }

  public DrivetrainSubsystem getDriveSubsystem(){
    return drivetrainSubsystem;
  }
}