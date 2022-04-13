// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.naming.PartialResultException;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.autonomous.AutoAlignAndDriveAndStop;
import frc.robot.commands.autonomous.AutoIntakeCommand;
import frc.robot.commands.autonomous.DriveToPointCommand;
import frc.robot.commands.autonomous.InvertPipelineCommand;
import frc.robot.commands.autonomous.MoveDistCommand;
import frc.robot.commands.autonomous.RotateByAngleCommand;
import frc.robot.commands.autonomous.RotateToAngleCommand;
import frc.robot.commands.autonomous.ShootOneBallCommand;
import frc.robot.commands.autonomous.StopWhenDist;
import frc.robot.commands.misc.AutoAlignAndDrive;
import frc.robot.commands.misc.AutoAlignCommand;
import frc.robot.commands.misc.AutoAlignShootCommand;
import frc.robot.commands.misc.OdometryCommand;
import frc.robot.commands.teleOp.DriveCommand;
import frc.robot.commands.teleOp.IntakeCommand;
import frc.robot.commands.teleOp.MoveWinchCommand;
import frc.robot.commands.teleOp.SecondaryClimberCommand;
import frc.robot.commands.teleOp.ClimberCommand;
import frc.robot.commands.teleOp.ConveyorCommand;
import frc.robot.commands.teleOp.ShooterCommand;
import frc.robot.commands.teleOp.ShooterMacro;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SecondaryClimberSystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WinchSubsystem;
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
  public OdometryCommand odometry = Constants.odometry;
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final SecondaryClimberSystem secondaryClimberSystem = new SecondaryClimberSystem();
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LEDSubsystem LEDS = new LEDSubsystem();
  
  private final Joystick leftJoystick = new Joystick(0);
  private final Joystick rightJoystick = new Joystick(1);
  public final Joystick buttonBoard = new Joystick(2);

  private final DriveCommand driveCommand = new DriveCommand(drivetrainSubsystem, leftJoystick, rightJoystick);
  private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem, buttonBoard);
  private final ConveyorCommand conveyorCommand = new ConveyorCommand(conveyorSubsystem, buttonBoard);
  private final ShooterCommand shooterCommand = new ShooterCommand(shooterSubsystem, buttonBoard);
  private final ClimberCommand climberCommand = new ClimberCommand(climberSubsystem, buttonBoard);
  private final SecondaryClimberCommand secondaryClimberCommand = new SecondaryClimberCommand(secondaryClimberSystem, buttonBoard);
  private final ShooterMacro shooterMacro = new ShooterMacro(shooterSubsystem, conveyorSubsystem);
  private final ShootOneBallCommand autonomousShoot = new ShootOneBallCommand(shooterSubsystem, conveyorSubsystem);
  public final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-fphil");
  public final NetworkTableEntry tx = table.getEntry("tx");
  public final NetworkTableEntry ty = table.getEntry("ty");
  public final NetworkTableEntry ta = table.getEntry("ta");
  private final JoystickButton autoAlignButton = new JoystickButton(leftJoystick, 6);
  private final JoystickButton autoAlignAndDriveButton = new JoystickButton(rightJoystick, 5);
  private final JoystickButton shootMacroButton = new JoystickButton(buttonBoard, 5);
  private final AutoAlignShootCommand autoAlignCommand = new AutoAlignShootCommand(drivetrainSubsystem);
  private final AutoAlignAndDrive autoAlignAndDrive = new AutoAlignAndDrive(drivetrainSubsystem);
  private final AutoAlignAndDriveAndStop autonomousIntake = new AutoAlignAndDriveAndStop(drivetrainSubsystem,odometry);
  private final MoveDistCommand autonomousDrive = new MoveDistCommand(86, 0.35, odometry, drivetrainSubsystem);
  private final MoveDistCommand autoDropForward = new MoveDistCommand(2, 0.35, odometry, drivetrainSubsystem);
  private final AutoIntakeCommand autoIntake = new AutoIntakeCommand(intakeSubsystem);
  private final WinchSubsystem winch = new WinchSubsystem();
  private final MoveWinchCommand winchCommand = new MoveWinchCommand(winch,leftJoystick,rightJoystick,buttonBoard);

  private Command teleOp;
  private Command autonomous;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    
    configureButtonBindings();
    teleOp = new ParallelCommandGroup(intakeCommand,winchCommand);
    PortForwarder.add(5800, "photonvision.local", 5800);
    LEDS.setBlinkin1Pattern(LEDStyleEnum.LIME.value);
    LEDS.setBlinkin2Pattern(LEDStyleEnum.LIME.value);
  }

  public LEDSubsystem getLEDS() {
      return LEDS;
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
    shootMacroButton.whenPressed(shooterMacro);
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

  public void setAutoCommand(AutoModeEnum mode, double delay) {
    if(mode == AutoModeEnum.ONE_BALL){
      autonomous = new SequentialCommandGroup(
        new ShootOneBallCommand(shooterSubsystem, conveyorSubsystem),
        new WaitCommand(delay),
        autonomousDrive
      );
    }else if(mode == AutoModeEnum.TWO_BALL){
      autonomous = new SequentialCommandGroup(
        new WaitCommand(delay),
        autoDropForward,
        new WaitCommand(1),
        new ParallelRaceGroup(
          new AutoAlignAndDriveAndStop(drivetrainSubsystem, odometry),
          new WaitCommand(3),
          new AutoIntakeCommand(intakeSubsystem)
        ),
        new ParallelRaceGroup(
          new AutoIntakeCommand(intakeSubsystem),
          new WaitCommand(0.25)
        ),
        new ParallelRaceGroup(
          new RotateToAngleCommand(drivetrainSubsystem, 0),
          new AutoIntakeCommand(intakeSubsystem),
          new WaitCommand(1)
        ),
        new ParallelRaceGroup(
          new AutoAlignShootCommand(drivetrainSubsystem),
          new AutoIntakeCommand(intakeSubsystem),
          new WaitCommand(1)
        ),
        new ParallelRaceGroup(
          new RotateByAngleCommand(drivetrainSubsystem, -20),
          new AutoIntakeCommand(intakeSubsystem),
          new WaitCommand(0.2)
        ),
        new ParallelRaceGroup(
          new MoveDistCommand(86, -0.25, odometry, drivetrainSubsystem),
          new AutoIntakeCommand(intakeSubsystem)
        ),
        new WaitCommand(0.1),
        new ShooterMacro(shooterSubsystem, conveyorSubsystem)
      );
    }else if(mode == AutoModeEnum.DEFENSE){
      autonomous = new SequentialCommandGroup(
        new WaitCommand(delay),
        autoDropForward,
        new WaitCommand(1),
        new ParallelRaceGroup(
          new AutoAlignAndDriveAndStop(drivetrainSubsystem, odometry),
          new WaitCommand(3),
          new AutoIntakeCommand(intakeSubsystem)
        ),
        new ParallelRaceGroup(
          new AutoIntakeCommand(intakeSubsystem),
          new WaitCommand(0.25)
        ),
        new ParallelRaceGroup(
          new RotateToAngleCommand(drivetrainSubsystem, 0),
          new AutoIntakeCommand(intakeSubsystem),
          new WaitCommand(1)
        ),
        new ParallelRaceGroup(
          new AutoAlignShootCommand(drivetrainSubsystem),
          new AutoIntakeCommand(intakeSubsystem),
          new WaitCommand(1)
        ),
        new ParallelRaceGroup(
          new RotateByAngleCommand(drivetrainSubsystem, -20),
          new AutoIntakeCommand(intakeSubsystem),
          new WaitCommand(0.2)
        ),
        new ParallelRaceGroup(
          new MoveDistCommand(86, -0.25, odometry, drivetrainSubsystem),
          new AutoIntakeCommand(intakeSubsystem)
        ),
        new WaitCommand(0.1),
        new ShooterMacro(shooterSubsystem, conveyorSubsystem),
        new MoveDistCommand(10, 0.25, odometry, drivetrainSubsystem),
        new ParallelRaceGroup(
          new RotateToAngleCommand(drivetrainSubsystem, -40),
          new WaitCommand(1)
        ),
        new InvertPipelineCommand(table.getEntry("pipeline").getDouble(0.0)),
        new WaitCommand(1),
        new ParallelRaceGroup(
          new AutoAlignAndDriveAndStop(drivetrainSubsystem, odometry),
          new WaitCommand(1.5),
          new AutoIntakeCommand(intakeSubsystem)
        ),
        new AutoIntakeCommand(intakeSubsystem)
      );
    }else if(mode == AutoModeEnum.THREE_BALL){
      autonomous = new SequentialCommandGroup(
        new ShootOneBallCommand(shooterSubsystem, conveyorSubsystem),
        autoDropForward,
        new MoveDistCommand(10, 0.35, odometry, drivetrainSubsystem),
        new ParallelRaceGroup(
          new RotateToAngleCommand(drivetrainSubsystem, -50),
          new WaitCommand(0.5)
        ),
        new ParallelRaceGroup(
          new AutoAlignAndDriveAndStop(drivetrainSubsystem, odometry),
          new WaitCommand(3),
          new AutoIntakeCommand(intakeSubsystem)
        ),
        new ParallelRaceGroup(
          new RotateToAngleCommand(drivetrainSubsystem, -80),
          new AutoIntakeCommand(intakeSubsystem),
          new WaitCommand(0.5)
        ),
        new ParallelRaceGroup(
          new MoveDistCommand(80, 0.35, odometry, drivetrainSubsystem),
          new AutoIntakeCommand(intakeSubsystem)
        ),
        new ParallelRaceGroup(
          new AutoAlignAndDrive(drivetrainSubsystem),
          new WaitCommand(3),
          new AutoIntakeCommand(intakeSubsystem),
          new StopWhenDist(242.837)
        ),
        new ParallelRaceGroup(
          new RotateToAngleCommand(drivetrainSubsystem, -100),
          new AutoIntakeCommand(intakeSubsystem),
          new WaitCommand(0.5)
        ),
        new ParallelRaceGroup(
          new MoveDistCommand(220, -0.35, odometry, drivetrainSubsystem),
          new AutoIntakeCommand(intakeSubsystem),
          new WaitCommand(0.5)
        ),
        new ParallelRaceGroup(
          new RotateToAngleCommand(drivetrainSubsystem, 0),
          new AutoIntakeCommand(intakeSubsystem),
          new WaitCommand(0.75)
        ),
        new ParallelRaceGroup(
          new AutoAlignShootCommand(drivetrainSubsystem),
          new WaitCommand(1)
        ),
        new MoveDistCommand(66, -0.25, odometry, drivetrainSubsystem),
        new ShooterMacro(shooterSubsystem, conveyorSubsystem)
      );
    }
  }

  public DrivetrainSubsystem getDriveSubsystem(){
    return drivetrainSubsystem;
  }

  public ConveyorSubsystem getConveyorSubsystem() {
      return conveyorSubsystem;
  }

  public ShooterSubsystem getShooterSubsystem() {
      return shooterSubsystem;
  }

  public ClimberSubsystem getClimberSubsystem() {
      return climberSubsystem;
  }
  
  public SecondaryClimberSystem getSecondaryClimberSystem() {
      return secondaryClimberSystem;
  }

  public ShooterCommand getShooterCommand() {
      return shooterCommand;
  }
  
  public ConveyorCommand getConveyorCommand() {
      return conveyorCommand;
  }
  
  public ClimberCommand getClimberCommand() {
      return climberCommand;
  }

  public SecondaryClimberCommand getSecondaryClimberCommand() {
      return secondaryClimberCommand;
  }
}