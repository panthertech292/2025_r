// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.ElevatorConstants.ElevatorHeights;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultElevator;
import frc.robot.commands.RotateGrabber;
import frc.robot.commands.SetElevatorSetPoint;
import frc.robot.commands.SetElevatorSpeed;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeOutputSubsystem;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final GrabberSubsystem m_GrabberSubsystem = new GrabberSubsystem();
  private final IntakeOutputSubsystem m_IntakeOutputSubsystem = new IntakeOutputSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final Command setElvatorUp = new SetElevatorSpeed(m_ElevatorSubsystem, .20);
  private final Command setElevatorDown = new SetElevatorSpeed(m_ElevatorSubsystem, -.20);

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    //Default commands
    m_ElevatorSubsystem.setDefaultCommand(new DefaultElevator(m_ElevatorSubsystem));
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        )
    );
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    m_driverController.y().whileTrue(setElvatorUp);
    m_driverController.a().whileTrue(setElevatorDown);
   // m_driverController.a().onTrue(new SetElevatorSetPoint(m_ElevatorSubsystem, ElevatorHeights.L3));
    //m_driverController.x().onTrue(new SetElevatorSetPoint(m_ElevatorSubsystem, ElevatorHeights.L4));
    m_driverController.rightBumper().whileTrue(new RotateGrabber(m_GrabberSubsystem, 0.1));
    m_driverController.leftBumper().whileTrue(new RotateGrabber(m_GrabberSubsystem, -0.1));
    m_driverController.x().whileTrue(Commands.startEnd(() -> m_IntakeOutputSubsystem.setBoth(.20), () -> m_IntakeOutputSubsystem.setBoth(0), m_IntakeOutputSubsystem));
    m_driverController.b().whileTrue(Commands.startEnd(() -> m_IntakeOutputSubsystem.setBoth(-.20), () -> m_IntakeOutputSubsystem.setBoth(0), m_IntakeOutputSubsystem));
    m_driverController.povUp().whileTrue(Commands.startEnd(() -> m_IntakeOutputSubsystem.setBothOpposite(.1, .7), () -> m_IntakeOutputSubsystem.setBoth(0), m_IntakeOutputSubsystem));
    m_driverController.povDown().whileTrue(Commands.startEnd(() -> m_IntakeOutputSubsystem.setBothOpposite(-.1, -.7), () -> m_IntakeOutputSubsystem.setBoth(0), m_IntakeOutputSubsystem));
    m_driverController.rightTrigger().whileTrue(Commands.startEnd(() -> m_GrabberSubsystem.setTranslation(.1), () -> m_GrabberSubsystem.setTranslation(0), m_GrabberSubsystem));
    m_driverController.leftTrigger().whileTrue(Commands.startEnd(() -> m_GrabberSubsystem.setTranslation(-.1), () -> m_GrabberSubsystem.setTranslation(0), m_GrabberSubsystem));

    m_driverController.leftStick().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
