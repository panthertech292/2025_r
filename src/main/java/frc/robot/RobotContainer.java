// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants.ElevatorHeights;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultElevator;
import frc.robot.commands.ElevatorSetSpeed;
import frc.robot.commands.GrabberSetRotateSpeed;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeOutputSubsystem;

public class RobotContainer {
  //Joysticks
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  //private final CommandXboxController operatorController = new CommandXboxController(1);
  //Subsystems
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final GrabberSubsystem m_GrabberSubsystem = new GrabberSubsystem();
  private final IntakeOutputSubsystem m_IntakeOutputSubsystem = new IntakeOutputSubsystem();

  //Drive Config
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public RobotContainer() {
    configureBindings();
    //Default commands
    m_ElevatorSubsystem.setDefaultCommand(new DefaultElevator(m_ElevatorSubsystem));
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
    
    //driverController.x().onTrue(Commands.runOnce(() -> m_ElevatorSubsystem.setElevatorSetPoint(ElevatorHeights.L3), m_ElevatorSubsystem)); untested
    
    //Test/Manual buttons
    driverController.rightBumper().whileTrue(new GrabberSetRotateSpeed(m_GrabberSubsystem, 0.1));
    driverController.leftBumper().whileTrue(new GrabberSetRotateSpeed(m_GrabberSubsystem, -0.1));
    driverController.y().whileTrue(new ElevatorSetSpeed(m_ElevatorSubsystem, 0.1));
    driverController.a().whileTrue(new ElevatorSetSpeed(m_ElevatorSubsystem, -0.1));
    driverController.x().whileTrue(Commands.startEnd(() -> m_IntakeOutputSubsystem.setBoth(.20, .20), () -> m_IntakeOutputSubsystem.setBoth(0, 0), m_IntakeOutputSubsystem));
    driverController.b().whileTrue(Commands.startEnd(() -> m_IntakeOutputSubsystem.setBoth(-.20, -.20), () -> m_IntakeOutputSubsystem.setBoth(0, 0), m_IntakeOutputSubsystem));
    driverController.povUp().whileTrue(Commands.startEnd(() -> m_IntakeOutputSubsystem.setBoth(.1, -.7), () -> m_IntakeOutputSubsystem.setBoth(0, 0), m_IntakeOutputSubsystem));
    driverController.povDown().whileTrue(Commands.startEnd(() -> m_IntakeOutputSubsystem.setBoth(-.1, .7), () -> m_IntakeOutputSubsystem.setBoth(0, 0), m_IntakeOutputSubsystem));
    driverController.rightTrigger().whileTrue(Commands.startEnd(() -> m_GrabberSubsystem.setTranslation(.1), () -> m_GrabberSubsystem.setTranslation(0), m_GrabberSubsystem));
    driverController.leftTrigger().whileTrue(Commands.startEnd(() -> m_GrabberSubsystem.setTranslation(-.1), () -> m_GrabberSubsystem.setTranslation(0), m_GrabberSubsystem));

    //driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    //driverController.b().whileTrue(drivetrain.applyRequest(() ->
    //point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.Note that each routine should be run exactly once in a single log.
    /*driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));*/

    // reset the field-centric heading on left bumper press
    //driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
      return Commands.print("No autonomous command configured");
  }
}
