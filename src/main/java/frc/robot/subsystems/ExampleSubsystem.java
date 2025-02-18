// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  //Encoders
  private final DutyCycleEncoder RotationAngleEncoder;
  private double encoderValue;
  private double previousEncoderValue;
  private int encoderRolloverOffset;
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    RotationAngleEncoder = new DutyCycleEncoder(0);
    encoderValue = getRotationAngle();
    encoderRolloverOffset = 0;
  }

  public double getRotationAngle(){
    return RotationAngleEncoder.get();
  }
  private double getTotalRotationAngle(){
    return getRotationAngle() + encoderRolloverOffset;
  }

  private void trackEncoder(){
    previousEncoderValue = encoderValue;
    encoderValue = getRotationAngle();
    double positionDelta = encoderValue - previousEncoderValue;
    if ((Math.abs(positionDelta)) > 0.50){
      System.out.println("We have rolled over!");
      System.out.println("Previous Value: " + previousEncoderValue + "    New Value: " + encoderValue);
      if(positionDelta < 0){ // Rolled forward
        encoderRolloverOffset++;
      }
      if(positionDelta > 0){ // Rolled backward
        encoderRolloverOffset--;
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(getRotationAngle());
    trackEncoder();
    System.out.println(getTotalRotationAngle());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
