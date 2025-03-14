// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GrabberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GrabberSetTranslate extends Command {
  private GrabberSubsystem GrabberSub;
  private double translateSpeed;
  /** Creates a new GrabberSetTranslate. */
  public GrabberSetTranslate(GrabberSubsystem Grabber_Subsystem, double speed) {
    GrabberSub = Grabber_Subsystem;
    translateSpeed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(GrabberSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    GrabberSub.setTranslation(translateSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    GrabberSub.setTranslation(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
