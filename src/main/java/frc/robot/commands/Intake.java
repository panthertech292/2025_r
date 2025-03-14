// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeOutputConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorHeights;
import frc.robot.Constants.GrabberConstants.GrabberLocations;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeOutputSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Intake extends Command {
  private ElevatorSubsystem ElevatorSub;
  private GrabberSubsystem GrabberSub;
  private IntakeOutputSubsystem IntakeOutputSub;
  /** Creates a new Intake. */
  public Intake(ElevatorSubsystem Elevator_Subsystem, GrabberSubsystem Grabber_Subsystem, IntakeOutputSubsystem IntakeOutput_Subsystem) {
    ElevatorSub = Elevator_Subsystem;
    GrabberSub = Grabber_Subsystem;
    IntakeOutputSub = IntakeOutput_Subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ElevatorSub, GrabberSub, IntakeOutputSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ElevatorSub.setElevatorSetPoint(ElevatorHeights.LOAD);
    GrabberSub.setGrabberPosition(GrabberLocations.LOAD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    GrabberSub.runGrabberFromSetAngleAndPosition();
    ElevatorSub.runElevatorFromSetHeight();
    IntakeOutputSub.setBoth(IntakeOutputConstants.kIntakeSpeed_ForIntake, IntakeOutputConstants.kIntakeSpeed_ForOutput);
    if(IntakeOutputSub.coralIsInOutput()){
      GrabberSub.setGrabberPosition(GrabberLocations.STOWED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeOutputSub.setBoth(0, 0);
    //GrabberSub.setGrabberPosition(GrabberLocations.STOWED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//return IntakeOutputSub.coralIsInOutput();
  }
}
