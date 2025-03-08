// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {
  private final TalonFXS rotateMotor;
  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem() {
    rotateMotor = new TalonFXS(30);
    TalonFXSConfiguration rotateMotorConfig = new TalonFXSConfiguration();
    rotateMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    rotateMotor.getConfigurator().apply(rotateMotorConfig);
  }

  public void setRotate(double speed){
    rotateMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
