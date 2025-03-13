// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX ClimberMotor;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    ClimberMotor = new TalonFX(ClimberConstants.kClimberMotor);
    TalonFXConfiguration climbConfig = new TalonFXConfiguration();
    climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ClimberMotor.getConfigurator().apply(climbConfig);
  }
  public void setClimb(double speed){
    ClimberMotor.set(speed);
  }
  public double getRelativePosition(){
    return ClimberMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
