// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorHeights;

public class ElevatorSubsystem extends SubsystemBase {
  private double elevatorSetHeight;
  private final TalonFX leftElevatorMotor;
  private final TalonFX rightElevatorMotor;
  private final CANdi elevatorCANdi;
  private final DutyCycleOut elevatorPower;
  private final MotionMagicVoltage elevatorMotionMagicVoltage;
  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {
    leftElevatorMotor = new TalonFX(ElevatorConstants.kLeftElevatorMotor);
    rightElevatorMotor = new TalonFX(ElevatorConstants.kRightElevatorMotor);
    elevatorCANdi = new CANdi(ElevatorConstants.kCANdi);
    elevatorPower = new DutyCycleOut(0.0);
    elevatorMotionMagicVoltage = new MotionMagicVoltage(0);
    //Left Motor Config
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leftConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.kElevatorStatorCurrentLimit;
    leftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANdiPWM1;
    leftConfig.Feedback.FeedbackRemoteSensorID = ElevatorConstants.kCANdi;
    leftConfig.Feedback.RotorToSensorRatio = ElevatorConstants.kIntEncoderToExtRatio;

    //MotionMagic
    leftConfig.Slot0.kG = 0.04;
    leftConfig.Slot0.kS = 0.35;
    leftConfig.Slot0.kV = 1.273;
    leftConfig.Slot0.kA = 0.0; 
    leftConfig.Slot0.kP = 100; 
    leftConfig.MotionMagic.MotionMagicCruiseVelocity = 4;
    leftConfig.MotionMagic.MotionMagicAcceleration = 8;
    leftConfig.MotionMagic.MotionMagicJerk = 80;

    leftElevatorMotor.getConfigurator().apply(leftConfig);
    //Right Motor Config
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.kElevatorStatorCurrentLimit;
    rightElevatorMotor.getConfigurator().apply(rightConfig);
    rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), true));
    //CANdi Config
    CANdiConfiguration CANdiConfig = new CANdiConfiguration();
    CANdiConfig.DigitalInputs.S1FloatState = S1FloatStateValue.FloatDetect;
    CANdiConfig.PWM1.AbsoluteSensorOffset = ElevatorConstants.kElevatorEncoderOffset;
    elevatorCANdi.getConfigurator().apply(CANdiConfig);
    
    BaseStatusSignal.setUpdateFrequencyForAll(100, elevatorCANdi.getPWM1Position());
  }

  public void setElevator(double speed){ //Note: These limits only check when called.
    leftElevatorMotor.setControl(elevatorPower.withOutput(speed)
    .withLimitForwardMotion(isElevatorMaxHeight())
    .withLimitReverseMotion(isElevatorMinHeight()));
  }
  public void setElevatorHeight(double height){
    if(height >= ElevatorConstants.kElevatorMinHeight){
      double rots = height / ElevatorConstants.kElevatorGearDiameter / Math.PI; //convert height to motor rotations
      leftElevatorMotor.setControl(elevatorMotionMagicVoltage.withPosition(rots)
      .withLimitForwardMotion(isElevatorMaxHeight())
      .withLimitReverseMotion(isElevatorMinHeight()));
    }else{
      leftElevatorMotor.set(0);
    }
  }
  public void runElevatorFromSetHeight(){
    setElevatorHeight(elevatorSetHeight);
  }
  public void setHeightToCurrentPosition(){
    elevatorSetHeight = getElevatorHeight();
  }

  public void setElevatorSetPoint(ElevatorHeights elevatorSetPoint){
    if(elevatorSetPoint == ElevatorHeights.DOWN){
      elevatorSetHeight = ElevatorConstants.kElevatorMinHeight;
    }
    if(elevatorSetPoint == ElevatorHeights.L1){
      elevatorSetHeight = ElevatorConstants.kElevatorMinHeight;
    }
    if(elevatorSetPoint == ElevatorHeights.L2){
      elevatorSetHeight = ElevatorConstants.kElevatorL2Height;
    }
    if(elevatorSetPoint == ElevatorHeights.L3){
      elevatorSetHeight = ElevatorConstants.kElevatorL3Height;
    }
    if(elevatorSetPoint == ElevatorHeights.ALGEE_LOW){
      elevatorSetHeight = ElevatorConstants.kElevatorAlgeeLowHeight;
    }
    if(elevatorSetPoint == ElevatorHeights.ALGEE_HIGH){
      elevatorSetHeight = ElevatorConstants.kElevatorAlgeeHighHeight;
    }
    if(elevatorSetPoint == ElevatorHeights.LOAD){
      elevatorSetHeight = ElevatorConstants.kElevatorLoadHeight;
    }
  }

  public boolean isElevatorAtHeight(){
    return (Math.abs(getElevatorHeight()-elevatorSetHeight) < .05);
  }

  public double getElevatorHeight(){ //Gets elevator height in inches
    return elevatorCANdi.getPWM1Position().getValueAsDouble() * ElevatorConstants.kElevatorGearDiameter * Math.PI ;
  }

  public boolean isElevatorMaxHeight(){
    return getElevatorHeight() > ElevatorConstants.kElevatorMaxHeight;
  }
  public boolean isElevatorMinHeight(){
    return getElevatorHeight() < ElevatorConstants.kElevatorMinHeight;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Rotation", elevatorCANdi.getPWM1Position().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Position", getElevatorHeight());
    //SmartDashboard.putNumber("Elevator Stator: " , leftElevatorMotor.getStatorCurrent().getValueAsDouble());
    //SmartDashboard.putNumber("Elevator Torque: ", leftElevatorMotor.getTorqueCurrent().getValueAsDouble());
    //SmartDashboard.putNumber("Left Elveator Voltage", leftElevatorMotor.getMotorVoltage().getValueAsDouble());
    //SmartDashboard.putNumber("Right Elveator Voltage", rightElevatorMotor.getMotorVoltage().getValueAsDouble());
    //SmartDashboard.putNumber("Kraken Internal Encoder", leftElevatorMotor.getPosition().getValueAsDouble());
    //System.out.println("Rotations Per Second: " + leftElevatorMotor.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
