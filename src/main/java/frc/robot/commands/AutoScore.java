// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorHeights;
import frc.robot.Constants.GrabberConstants.GrabberLocations;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeOutputSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScore extends Command {
  private ElevatorSubsystem ElevatorSub;
  private GrabberSubsystem GrabberSub;
  private IntakeOutputSubsystem IntakeOutputSub;
  private ElevatorHeights height;
  private GrabberLocations rotation;
  private boolean scoringLeft;
  /** Creates a new AutoScore. */
  public AutoScore(ElevatorSubsystem Elevator_Subsystem, GrabberSubsystem Grabber_Subsystem, IntakeOutputSubsystem IntakeOutput_Subsystem, ElevatorHeights setHeight, GrabberLocations setRotation, boolean scoreLeft) {
    ElevatorSub = Elevator_Subsystem;
    GrabberSub = Grabber_Subsystem;
    IntakeOutputSub = IntakeOutput_Subsystem;
    height = setHeight;
    rotation = setRotation;
    scoringLeft = scoreLeft;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ElevatorSub, GrabberSub, IntakeOutputSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ElevatorSub.setElevatorSetPoint(height);
    GrabberSub.setGrabberPosition(rotation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    GrabberSub.runGrabberFromSetAngleAndPosition();
    ElevatorSub.runElevatorFromSetHeight();
    if(ElevatorSub.isElevatorAtHeight()){
      if(scoringLeft){
        IntakeOutputSub.setBoth(-.20, -.20);
      }else{
        IntakeOutputSub.setBoth(.20, .20);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ElevatorSub.setElevatorSetPoint(ElevatorHeights.DOWN);
    GrabberSub.setGrabberPosition(GrabberLocations.LOAD);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
