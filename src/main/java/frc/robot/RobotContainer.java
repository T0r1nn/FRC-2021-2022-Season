// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.autonomous.AutoAlignAndDriveAndStop;
import frc.robot.commands.autonomous.AutoIntakeCommand;
import frc.robot.commands.autonomous.InvertPipelineCommand;
import frc.robot.commands.autonomous.MoveDistCommand;
import frc.robot.commands.autonomous.RotateByAngleCommand;
import frc.robot.commands.autonomous.RotateToAngleCommand;
import frc.robot.commands.autonomous.ShootOneBallCommand;
import frc.robot.commands.autonomous.StopWhenDist;
import frc.robot.commands.jjSummerBash.RotateCommand;
import frc.robot.commands.jjSummerBash.RunIntakeCommand;
import frc.robot.commands.jjSummerBash.FullShootCommand;
import frc.robot.commands.misc.AutoAlignAndDrive;
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
  //Gets the odometry system
  public OdometryCommand odometry = Constants.odometry;
  
  //A subsystem to control the drivetrain
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  //A subsystem to control the intake
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  //A subsystem to control the primary climber
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  //A subsystem to control the secondary climber
  private final SecondaryClimberSystem secondaryClimberSystem = new SecondaryClimberSystem();
  //A subsystem to control the conveyor
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  //A subsystem to control the shooter
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  //A subsystem to control the LEDS
  private final LEDSubsystem LEDS = new LEDSubsystem();
  
  //Defining the three peripherals
  private final Joystick leftJoystick = new Joystick(0);
  private final Joystick rightJoystick = new Joystick(1);
  public final Joystick buttonBoard = new Joystick(2);

  //A command to drive the robot in teleop
  private final DriveCommand driveCommand = new DriveCommand(drivetrainSubsystem, leftJoystick, rightJoystick);
  //A command to intake during teleop
  private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem, buttonBoard);
  //A command to run the conveyor in teleop
  private final ConveyorCommand conveyorCommand = new ConveyorCommand(conveyorSubsystem, buttonBoard);
  //A command to run the shooter in teleop
  private final ShooterCommand shooterCommand = new ShooterCommand(shooterSubsystem, buttonBoard);
  //Two commands to run the climbers in teleop
  private final ClimberCommand climberCommand = new ClimberCommand(climberSubsystem, buttonBoard);
  private final SecondaryClimberCommand secondaryClimberCommand = new SecondaryClimberCommand(secondaryClimberSystem, buttonBoard);
  //A command to run the conveyor and shooter for enough time to shoot two cargo
  private final ShooterMacro shooterMacro = new ShooterMacro(shooterSubsystem, conveyorSubsystem);
  //The way the limelight is accessed
  public final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-fphil");
  //The limelight data
  public final NetworkTableEntry tx = table.getEntry("tx");
  public final NetworkTableEntry ty = table.getEntry("ty");
  public final NetworkTableEntry ta = table.getEntry("ta");
  //Joystick buttons to run macros
  private final JoystickButton autoAlignButton = new JoystickButton(leftJoystick, 6);
  private final JoystickButton autoAlignAndDriveButton = new JoystickButton(rightJoystick, 5);
  private final JoystickButton shootMacroButton = new JoystickButton(buttonBoard, 5);
  //A command to align to the upper hub during auto
  private final AutoAlignShootCommand autoAlignCommand = new AutoAlignShootCommand(drivetrainSubsystem);
  //Chases a cargo
  private final AutoAlignAndDrive autoAlignAndDrive = new AutoAlignAndDrive(drivetrainSubsystem);
  //Moves a certain distance, first one for taxiing and second one for dropping the intake
  private final MoveDistCommand autonomousDrive = new MoveDistCommand(86, 0.35, odometry, drivetrainSubsystem);
  private final MoveDistCommand autoDropForward = new MoveDistCommand(2, 0.35, odometry, drivetrainSubsystem);
  //The subsystem and command to control the winch during teleop
  private final WinchSubsystem winch = new WinchSubsystem();
  private final MoveWinchCommand winchCommand = new MoveWinchCommand(winch,leftJoystick,rightJoystick,buttonBoard);

  //J+J summer bash stuff
  private final FullShootCommand shootCommand = new FullShootCommand(intakeSubsystem, conveyorSubsystem, shooterSubsystem);
  private final RotateCommand rotateCommand = new RotateCommand(drivetrainSubsystem, rightJoystick);
  private final RunIntakeCommand runIntakeCommand = new RunIntakeCommand(intakeSubsystem);
  private final JoystickButton intake = new JoystickButton(buttonBoard, 11);

  //Teleop and autonomous command
  private Command teleOp;
  private Command autonomous;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //Creates teleop command
    teleOp = new ParallelCommandGroup(intakeCommand,winchCommand);
    //Sets up photonvision if we use that ever
    PortForwarder.add(5800, "photonvision.local", 5800);
    //Sets the LED colors
    LEDS.setBlinkin1Pattern(LEDStyleEnum.LIME.value);
    LEDS.setBlinkin2Pattern(LEDStyleEnum.LIME.value);
  }

  //Gets the leds
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
    //Configures the buttons
    // autoAlignButton.whenHeld(autoAlignCommand);
    // autoAlignAndDriveButton.whenHeld(autoAlignAndDrive);
    // shootMacroButton.whenPressed(shooterMacro);
    shootMacroButton.whenHeld(shooterMacro);
  }

  public void setAutoCommand(AutoModeEnum mode, double delay) {
    //Sets the auto command based on shuffleboard settings
    if(mode == AutoModeEnum.ONE_BALL){
      //Shoots, delays, then taxis
      autonomous = new SequentialCommandGroup(
        new ShootOneBallCommand(shooterSubsystem, conveyorSubsystem),
        new WaitCommand(delay),
        autonomousDrive
      );
    }else if(mode == AutoModeEnum.TWO_BALL){
      //Intakes another ball, drive back, and double shoot
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
      //Same as two ball but intakes a opposition cargo after
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
      //Shoots, double intakes, then double shoots
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

  //Command and subsystem getters

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

  public RotateCommand getRotateCommand(){
      return rotateCommand;
  }
}