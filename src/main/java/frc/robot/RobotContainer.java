// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants.ElevatorHeights;
import frc.robot.Constants.GrabberConstants.GrabberLocations;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoScore;
import frc.robot.commands.ClimbToPosition;
import frc.robot.commands.DefaultElevator;
import frc.robot.commands.DefaultGrabber;
import frc.robot.commands.ElevatorSetSpeed;
import frc.robot.commands.GrabberSetRotateSpeed;
import frc.robot.commands.GrabberSetTranslate;
import frc.robot.commands.Intake;
import frc.robot.commands.IntakeNoRotate;
import frc.robot.commands.SetupScore;
import frc.robot.commands.driveForwardUntilPost;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeOutputSubsystem;

public class RobotContainer {
  //Joysticks
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  //Subsystems
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final GrabberSubsystem m_GrabberSubsystem = new GrabberSubsystem();
  private final IntakeOutputSubsystem m_IntakeOutputSubsystem = new IntakeOutputSubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();

  //Drive Config
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.RobotCentric driveForward = new SwerveRequest.RobotCentric()
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    //CameraServer.startAutomaticCapture();
    configureBindings();
    registerCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto", autoChooser);
    //Default commands
    m_ElevatorSubsystem.setDefaultCommand(new DefaultElevator(m_ElevatorSubsystem));
    m_GrabberSubsystem.setDefaultCommand(new DefaultGrabber(m_GrabberSubsystem));
  }
  private void registerCommands(){
    NamedCommands.registerCommand("AutoScoreL2-Left", new AutoScore(m_ElevatorSubsystem, m_GrabberSubsystem, m_IntakeOutputSubsystem, ElevatorHeights.L2, GrabberLocations.L2, true));
    NamedCommands.registerCommand("AutoScoreL2-Right", new AutoScore(m_ElevatorSubsystem, m_GrabberSubsystem, m_IntakeOutputSubsystem, ElevatorHeights.L2, GrabberLocations.L2, false));
    NamedCommands.registerCommand("Intake", new Intake(m_ElevatorSubsystem, m_GrabberSubsystem, m_IntakeOutputSubsystem));
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
    // Drivetrain will execute this command periodically
    drivetrain.applyRequest(() ->
    drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    .withRotationalRate(-driverController.getRightX() * MaxAngularRate))); // Drive counterclockwise with negative X (left)
    //Driver Conroller
    driverController.rightBumper().whileTrue(drivetrain.applyRequest(() ->
    driveForward.withVelocityX(.5) // Drive forward with negative Y (forward)
    .withVelocityY(0) // Drive left with negative X (left)
    .withRotationalRate(0)).raceWith(new driveForwardUntilPost(m_IntakeOutputSubsystem)));
    driverController.leftBumper().whileTrue(drivetrain.applyRequest(() ->
    driveForward.withVelocityX(-.5).withVelocityY(0).withRotationalRate(0)).raceWith(new driveForwardUntilPost(m_IntakeOutputSubsystem)));
    driverController.b().whileTrue(drivetrain.applyRequest(() ->
    drive.withVelocityX(-driverController.getLeftY() * MaxSpeed*0.15) // Drive forward with negative Y (forward)
    .withVelocityY(-driverController.getLeftX() * MaxSpeed*0.15) // Drive left with negative X (left)
    .withRotationalRate(-driverController.getRightX() * MaxAngularRate*0.15)));
    driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    driverController.rightTrigger().whileTrue(Commands.startEnd(() -> m_IntakeOutputSubsystem.setBoth(-.20, -.20), () -> m_IntakeOutputSubsystem.setBoth(0, 0), m_IntakeOutputSubsystem));
    driverController.leftTrigger().and(driverController.rightTrigger()).whileTrue(Commands.startEnd(() -> m_IntakeOutputSubsystem.setBoth(.20, .20), () -> m_IntakeOutputSubsystem.setBoth(0, 0), m_IntakeOutputSubsystem));
    driverController.a().whileTrue(drivetrain.driveToPose(new Pose2d(0.851, 6.746, new Rotation2d(148.627))));
    //Operator Controller
    operatorController.povUp().whileTrue(new ClimbToPosition(m_ClimberSubsystem, 0.25, 9999999));
    operatorController.a().whileTrue(new Intake(m_ElevatorSubsystem, m_GrabberSubsystem, m_IntakeOutputSubsystem));
    operatorController.start().whileTrue(new IntakeNoRotate(m_ElevatorSubsystem, m_GrabberSubsystem, m_IntakeOutputSubsystem));
    operatorController.b().onTrue(new SetupScore(m_ElevatorSubsystem, m_GrabberSubsystem, ElevatorHeights.L2, GrabberLocations.L2));
    operatorController.y().onTrue(new SetupScore(m_ElevatorSubsystem, m_GrabberSubsystem, ElevatorHeights.L3, GrabberLocations.L3));
    operatorController.leftBumper().whileTrue(new GrabberSetRotateSpeed(m_GrabberSubsystem, -0.1));
    operatorController.rightBumper().whileTrue(new GrabberSetRotateSpeed(m_GrabberSubsystem, 0.1));
    operatorController.leftTrigger().whileTrue(new ElevatorSetSpeed(m_ElevatorSubsystem, -0.25));
    operatorController.rightTrigger().whileTrue(new ElevatorSetSpeed(m_ElevatorSubsystem, 0.25));

    //driverController.rightBumper().onTrue(Commands.runOnce(() -> m_ElevatorSubsystem.setElevatorSetPoint(ElevatorHeights.L3), m_ElevatorSubsystem));
    //Test/Manual buttons
    //driverController.rightBumper().whileTrue(new GrabberSetRotateSpeed(m_GrabberSubsystem, 0.1));
    //driverController.leftBumper().whileTrue(new GrabberSetRotateSpeed(m_GrabberSubsystem, -0.1));
    //driverController.povUp().onTrue(Commands.runOnce(() -> m_GrabberSubsystem.setGrabberPosition(GrabberLocations.STOWED), m_GrabberSubsystem));
    //driverController.povRight().onTrue(Commands.runOnce(() -> m_GrabberSubsystem.setGrabberPosition(GrabberLocations.LOAD), m_GrabberSubsystem));
    //driverController.povLeft().onTrue(Commands.runOnce(() -> m_GrabberSubsystem.setGrabberPosition(GrabberLocations.L2), m_GrabberSubsystem));
    //driverController.back().whileTrue(new ClimbToPosition(m_ClimberSubsystem, -0.15, 9999999));

    //driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    //driverController.b().whileTrue(drivetrain.applyRequest(() ->
    //point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.Note that each routine should be run exactly once in a single log.
    /*driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));*/
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
      return autoChooser.getSelected();
  }
}
