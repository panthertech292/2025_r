// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeOutputConstants;

public class IntakeOutputSubsystem extends SubsystemBase {
  private final TalonFXS intakeMotor;
  private final TalonFXS outputMotor;
  private final CANdi GrabberCANdi;
  /** Creates a new IntakeOutputSubsystem. */
  public IntakeOutputSubsystem() {
    //CANdi Config
    GrabberCANdi = new CANdi(IntakeOutputConstants.kCANdi);
    CANdiConfiguration CANdiConfig = new CANdiConfiguration();
    CANdiConfig.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenLow;
    GrabberCANdi.getConfigurator().apply(CANdiConfig);
    //Intake Motor Config
    intakeMotor = new TalonFXS(IntakeOutputConstants.kIntakeMotor);
    TalonFXSConfiguration intakeMotorConfig = new TalonFXSConfiguration();
    intakeMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeOutputConstants.kSupplyCurrentLimit;
    intakeMotor.getConfigurator().apply(intakeMotorConfig);
    //Output Motor Config
    outputMotor = new TalonFXS(IntakeOutputConstants.kOutputMotor);
    TalonFXSConfiguration outputMotorConfig = new TalonFXSConfiguration();
    outputMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    outputMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    outputMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    outputMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeOutputConstants.kSupplyCurrentLimit;
    outputMotor.getConfigurator().apply(outputMotorConfig);
  }
  public void setIntake(double speed){
    intakeMotor.set(speed);
  }
  public void setOutput(double speed){
    outputMotor.set(speed);
  }
  public void setBoth(double intakeSpeed, double outputSpeed){
    setIntake(intakeSpeed);
    setOutput(outputSpeed);
  }
  public boolean coralIsInOutput(){
    return GrabberCANdi.getS1Closed().getValue();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
