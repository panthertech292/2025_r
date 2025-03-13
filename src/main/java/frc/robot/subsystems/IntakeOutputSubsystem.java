// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.IntakeOutputConstants;

public class IntakeOutputSubsystem extends SubsystemBase {
  private final TalonFXS intakeMotor;
  private final TalonFXS outputMotor;
  /** Creates a new IntakeOutputSubsystem. */
  public IntakeOutputSubsystem() {
    intakeMotor = new TalonFXS(IntakeOutputConstants.kIntakeMotor);
    TalonFXSConfiguration intakeMotorConfig = new TalonFXSConfiguration();
    intakeMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeOutputConstants.kStatorCurrentLimit;
    intakeMotor.getConfigurator().apply(intakeMotorConfig);

    outputMotor = new TalonFXS(IntakeOutputConstants.kOutputMotor);
    TalonFXSConfiguration outputMotorConfig = new TalonFXSConfiguration();
    outputMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    outputMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    outputMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    outputMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeOutputConstants.kStatorCurrentLimit;
    outputMotor.getConfigurator().apply(outputMotorConfig);
  }
  public void setIntake(double speed){
    intakeMotor.set(speed);
  }
  public void setOutput(double speed){
    outputMotor.set(speed);
  }

  public void setBoth(double speed){
    setIntake(speed);
    setOutput(speed);
  }
  public void setBothOpposite(double intakeSpeed, double outputSpeed){
    setIntake(intakeSpeed);
    setOutput(-outputSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Output Stator Current", outputMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Input Stator Current", intakeMotor.getStatorCurrent().getValueAsDouble());
  }
}
