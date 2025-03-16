// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.GrabberConstants.GrabberLocations;

public class GrabberSubsystem extends SubsystemBase {
  private final TalonFXS rotateMotor;
  private final TalonFXS translationMotor;
  private final CANdi GrabberCANdi;
  private final MotionMagicVoltage rotateMotionMagicVoltage;
  private final MotionMagicVoltage translationMotionMagicVoltage;
  private final DutyCycleOut rotatePower;
  private final DutyCycleOut translationPower;
  private double rotationAngle;
  private double translationPosition;
  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem() {
    
    //CANdi Config
    GrabberCANdi = new CANdi(GrabberConstants.kCANdi);
    CANdiConfiguration CANdiConfig = new CANdiConfiguration();
    CANdiConfig.DigitalInputs.S1FloatState = S1FloatStateValue.FloatDetect;
    CANdiConfig.PWM1.AbsoluteSensorOffset = GrabberConstants.kRotateEncoderOffset;
    CANdiConfig.DigitalInputs.S2FloatState = S2FloatStateValue.FloatDetect;
    CANdiConfig.PWM2.AbsoluteSensorOffset = GrabberConstants.kTranslationEncoderOffset;
    GrabberCANdi.getConfigurator().apply(CANdiConfig);
    
    //Rotate Motor Config
    rotateMotor = new TalonFXS(GrabberConstants.kRotateMotor);
    rotatePower = new DutyCycleOut(0);
    TalonFXSConfiguration rotateMotorConfig = new TalonFXSConfiguration();
    rotateMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    rotateMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rotateMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rotateMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rotateMotorConfig.CurrentLimits.StatorCurrentLimit = GrabberConstants.kRotateStatorCurrentLimit;
    rotateMotorConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.FusedCANdiPWM1;
    rotateMotorConfig.ExternalFeedback.FeedbackRemoteSensorID = GrabberConstants.kCANdi;
    rotateMotorConfig.ExternalFeedback.RotorToSensorRatio = GrabberConstants.kRotateIntEncoderToExtRatio;
    
    //Rotate Motor MotionMagic Config
    rotateMotionMagicVoltage = new MotionMagicVoltage(0);
    //rotateMotorConfig.Slot0.kG = 0;
    rotateMotorConfig.Slot0.kS = 0.27;
    rotateMotorConfig.Slot0.kV = 2; //Recalc suggests a value of 2.00
    rotateMotorConfig.Slot0.kA = 0; //TODO: Tune this
    rotateMotorConfig.Slot0.kP = 100; //TODO: Tune this
    //rotateMotorConfig.Slot0.kI = 0;
    //rotateMotorConfig.Slot0.kD = 0;
    rotateMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 4;
    rotateMotorConfig.MotionMagic.MotionMagicAcceleration = 8;
    rotateMotorConfig.MotionMagic.MotionMagicJerk = 80;

    rotateMotor.getConfigurator().apply(rotateMotorConfig);
    //Translation motor config
    translationMotor = new TalonFXS(GrabberConstants.kTranslationMotor);
    translationPower = new DutyCycleOut(0);
    TalonFXSConfiguration translationMotorConfig = new TalonFXSConfiguration();
    translationMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    translationMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    //translationMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    //translationMotorConfig.CurrentLimits.StatorCurrentLimit = GrabberConstants.kTranslateStatorCurrentLimit;
    translationMotorConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANdiPWM2;
    translationMotorConfig.ExternalFeedback.FeedbackRemoteSensorID = GrabberConstants.kCANdi;
    translationMotorConfig.ExternalFeedback.RotorToSensorRatio = GrabberConstants.kTranslateIntEncoderToExtRatio;

    //Translate Motor MotionMagic Config
    translationMotionMagicVoltage = new MotionMagicVoltage(0);
    //translationMotorConfig.Slot0.kG = 0;
    translationMotorConfig.Slot0.kS = 0.48;
    translationMotorConfig.Slot0.kV = 0.1; //Recalc suggests 2.44
    translationMotorConfig.Slot0.kA = 0; //TODO: Tune this 0.02
    translationMotorConfig.Slot0.kP = 10; //TODO: Tune this
    translationMotorConfig.Slot0.kI = 0;
    translationMotorConfig.Slot0.kD = 0;
    translationMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 2;
    translationMotorConfig.MotionMagic.MotionMagicAcceleration = 4;
    translationMotorConfig.MotionMagic.MotionMagicJerk = 40;
    translationMotor.getConfigurator().apply(translationMotorConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(100, GrabberCANdi.getPWM1Position(), GrabberCANdi.getPWM2Position());
    rotationAngle = getRotationAngle();
  }

  public void setRotate(double speed){
    rotateMotor.setControl(rotatePower.withOutput(speed)
    .withLimitForwardMotion(isRotationMaxClockwise())
    .withLimitReverseMotion(isRotationMinCounterClockwise()));
  }
  public void setRotateAngle(double angle){
    rotateMotor.setControl(rotateMotionMagicVoltage.withPosition(angle)
    .withLimitForwardMotion(isRotationMaxClockwise())
    .withLimitReverseMotion(isRotationMinCounterClockwise()));
  }
  public double getRotationAngle(){
    return GrabberCANdi.getPWM1Position().getValueAsDouble();
  }
  public boolean isRotationMaxClockwise(){
    return getRotationAngle() > GrabberConstants.kRotationMaxClockwise;
  }
  public boolean isRotationMinCounterClockwise(){
    return getRotationAngle() < GrabberConstants.kRotationMinCounterClockwise;
  }
  public void setTranslation(double speed){
    translationMotor.setControl(translationPower.withOutput(speed)
    .withLimitForwardMotion(isTranslationMaxRight())
    .withLimitReverseMotion(isTranslationMaxLeft()));
  }
  public void setTranslationDistance(double distance){
    translationMotor.setControl(translationMotionMagicVoltage.withPosition(distance)
    .withLimitForwardMotion(isTranslationMaxRight())
    .withLimitReverseMotion(isTranslationMaxLeft()));
  }
  public double getTranslationDistance(){
    return GrabberCANdi.getPWM2Position().getValueAsDouble();
  }
  public boolean isTranslationMaxLeft(){
    return getTranslationDistance() < GrabberConstants.KTranslationMaxLeft;
  }
  public boolean isTranslationMaxRight(){
    return getTranslationDistance() > GrabberConstants.kTranslationMaxRight;
  }
  public void runGrabberFromSetAngleAndPosition(){
    //setTranslationDistance(translationPosition);
    setRotateAngle(rotationAngle);
  }
  public void setRotationToCurrentPosition(){
    rotationAngle = getRotationAngle();
  }
  public void setGrabberPosition(GrabberLocations position){
    if(position == GrabberLocations.STOWED){
      rotationAngle = GrabberConstants.kRotationStowed;
      translationPosition = GrabberConstants.kTranslationStowed;
    }
    if(position == GrabberLocations.L1){
      rotationAngle = GrabberConstants.kRotationL1;
      translationPosition = GrabberConstants.KTranslationMaxLeft;
    }
    if(position == GrabberLocations.L2){
      rotationAngle = GrabberConstants.kRotationL2;
      translationPosition = GrabberConstants.kTranslationMaxRight;
    }
    if(position == GrabberLocations.L3){
      rotationAngle = GrabberConstants.kRotationL3;
      translationPosition = GrabberConstants.KTranslationMaxLeft;
    }
    if(position == GrabberLocations.ALGEE_LOW){
      rotationAngle = GrabberConstants.kRotationAlgeeLow;
      translationPosition = GrabberConstants.KTranslationMaxLeft;
    }
    if(position == GrabberLocations.ALGEE_HIGH){
      rotationAngle = GrabberConstants.kRotationAlgeeHigh;
      translationPosition = GrabberConstants.KTranslationMaxLeft;
    }
    if(position == GrabberLocations.LOAD){
      rotationAngle = GrabberConstants.kRotationLoad;
      translationPosition = GrabberConstants.KTranslationMaxLeft;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Grabber Rotation", getRotationAngle());
    //SmartDashboard.putNumber("Translation Rotation", getTranslationDistance());
  }
}
