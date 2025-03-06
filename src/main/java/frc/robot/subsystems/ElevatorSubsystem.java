// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX leftElevatorMotor;
  private final TalonFX rightElevatorMotor;
  private final CANdi elevatorCANdi;
  private final DutyCycleOut elevatorPower;
  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {
    leftElevatorMotor = new TalonFX(ElevatorConstants.kLeftElevatorMotor);
    rightElevatorMotor = new TalonFX(ElevatorConstants.kRightElevatorMotor);
    elevatorCANdi = new CANdi(ElevatorConstants.kCANdi);
    elevatorPower = new DutyCycleOut(0.0);

    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leftConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.kElevatorStatorCurrentLimit;
    leftElevatorMotor.getConfigurator().apply(leftConfig);

    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.kElevatorStatorCurrentLimit;
    rightElevatorMotor.getConfigurator().apply(rightConfig);

    CANdiConfiguration CANdiConfig = new CANdiConfiguration();
    CANdiConfig.DigitalInputs.S1FloatState = S1FloatStateValue.FloatDetect;
    CANdiConfig.PWM1.AbsoluteSensorOffset = ElevatorConstants.kElevatorEncoderOffset;
    elevatorCANdi.getConfigurator().apply(CANdiConfig);
    
    BaseStatusSignal.setUpdateFrequencyForAll(100, elevatorCANdi.getPWM1Position());

    rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), true));
  }

  public void setElevator(double speed){ //Note: These limits only check when called.
    leftElevatorMotor.setControl(elevatorPower.withOutput(speed)
    .withLimitForwardMotion(isElevatorMaxHeight())
    .withLimitReverseMotion(isElevatorMinHeight()));
  }

  public double getElevatorHeight(){ //Gets elevator height in inches
    return elevatorCANdi.getPWM1Position().getValueAsDouble() * ElevatorConstants.kElevatorGearDiameter * Math.PI ;
  }

  public boolean isElevatorMaxHeight(){
    return getElevatorHeight() > ElevatorConstants.kElevatorMaxHeight-10;
  }
  public boolean isElevatorMinHeight(){
    return getElevatorHeight() < ElevatorConstants.kElevatorMinHeight; //TODO: Change this value
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Rotation", elevatorCANdi.getPWM1Position().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Position", getElevatorHeight());
    SmartDashboard.putNumber("Elevator Stator: " , leftElevatorMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Torque: ", leftElevatorMotor.getTorqueCurrent().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
