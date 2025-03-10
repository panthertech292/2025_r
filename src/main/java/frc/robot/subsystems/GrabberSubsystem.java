// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class GrabberSubsystem extends SubsystemBase {
  private final TalonFXS rotateMotor;
  private final MotionMagicVoltage rotateMotionMagicVoltage;
  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem() {
    //Rotate Motor Config
    rotateMotor = new TalonFXS(GrabberConstants.kRotateMotor);
    TalonFXSConfiguration rotateMotorConfig = new TalonFXSConfiguration();
    rotateMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    rotateMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rotateMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rotateMotorConfig.CurrentLimits.StatorCurrentLimit = GrabberConstants.kRotateStatorCurrentLimit;
    rotateMotorConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.FusedCANdiPWM1; //TODO: Figure if this PWM1 or PWM2
    rotateMotorConfig.ExternalFeedback.FeedbackRemoteSensorID = GrabberConstants.kCANdiMotorEncoders;
    rotateMotorConfig.ExternalFeedback.RotorToSensorRatio = GrabberConstants.kRotateIntEncoderToExtRatio;
    

    //Rotate Motor MotionMagic Config
    rotateMotionMagicVoltage = new MotionMagicVoltage(0);
    rotateMotorConfig.Slot0.kG = 0;
    rotateMotorConfig.Slot0.kS = 0;
    rotateMotorConfig.Slot0.kV = 0; //Recalc suggests a value of 2.00
    rotateMotorConfig.Slot0.kA = 0; //TODO: Tune this
    rotateMotorConfig.Slot0.kP = 0; //TODO: Tune this
    rotateMotorConfig.Slot0.kI = 0;
    rotateMotorConfig.Slot0.kD = 0;
    rotateMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
    rotateMotorConfig.MotionMagic.MotionMagicAcceleration = 0;
    rotateMotorConfig.MotionMagic.MotionMagicJerk = 0;

    rotateMotor.getConfigurator().apply(rotateMotorConfig);
  }

  public void setRotate(double speed){
    rotateMotor.set(speed);
  }

  public void setRotateAngle(double angle){
    rotateMotor.setControl(rotateMotionMagicVoltage.withPosition(angle));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
