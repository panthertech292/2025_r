// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kFirstButtonBoardPort = 2;
    public static final int kSecondButtonBoardPort = 3;
  }
  public static class ElevatorConstants {
    public static enum ElevatorHeights{DOWN, L1, L2, L3, ALGEE_LOW, ALGEE_HIGH, LOAD};
    //Devices
    public static final int kLeftElevatorMotor = 20;
    public static final int kRightElevatorMotor = 21;
    public static final int kCANdi = 22;
    //Ratios & Offsets
    public static final double kElevatorGearDiameter = 2.551;
    public static final int kIntEncoderToExtRatio = 64;
    public static final double kElevatorEncoderOffset = -0.38232421875;
    //Current Limits
    public static final double kElevatorStatorCurrentLimit = 20; //TODO: Tune this
    //Limits
    public static final double kElevatorMaxHeight = 23.5;
    public static final double kElevatorMinHeight = 0.25;
    //Heights
    public static final double kElevatorL1Height = 0.25;
    public static final double kElevatorL2Height = 2.89; 
    public static final double kElevatorL3Height = 19.3; 
    public static final double kElevatorAlgeeLowHeight = 0.25; //TODO: Tune this
    public static final double kElevatorAlgeeHighHeight = 0.25; //TODO: Tune this
    public static final double kElevatorLoadHeight = 0.25;
  }
  public static class GrabberConstants {
    public static enum GrabberLocations{STOWED,L1, L2, L3, ALGEE_LOW, ALGEE_HIGH, LOAD}
    //Devices
    public static final int kRotateMotor = 30;
    public static final int kTranslationMotor = 31;
    public static final int kCANdi = 32;
    //Ratios & Offsets
    public static final double kRotateIntEncoderToExtRatio = 16 * (145/18); // (Should be 128.89). 16 is from gearbox, 145/18 is the pulley. 
    public static final double kTranslateIntEncoderToExtRatio = 16; //16 from gearbox
    public static final double kTranslatePulleyDiameter = 0; //TODO: Find this!
    public static final double kRotateEncoderOffset = 0.1533203125;
    public static final double kTranslationEncoderOffset = -0.651;
    //Current Limits
    public static final double kRotateStatorCurrentLimit = 20; //TODO: Tune this!
    public static final double kTranslateStatorCurrentLimit = 40; //TODO: Tune this
    //Limits
    public static final double kRotationMaxClockwise = 0.125; //.25 full limit
    public static final double kRotationMinCounterClockwise = -0.125; //-.24 full limit
    public static final double KTranslationMaxLeft = 0;
    public static final double kTranslationMaxRight = 0;
    public static final double kTranslationStowed = -0.11;
    //Positions
    public static final double kRotationStowed = 0;
    public static final double kRotationL1 = 0; //TODO: Find this!
    public static final double kRotationL2 = 0;//-0.04;
    public static final double kRotationL3 = 0;//-0.02; //TODO: Find this!
    public static final double kRotationAlgeeLow = 0; //TODO: Find this!
    public static final double kRotationAlgeeHigh = 0; //TODO: Find this!
    public static final double kRotationLoad = -0.088;//-0.053;
  }
  public static class IntakeOutputConstants{
    //Devices
    public static final int kIntakeMotor = 51;
    public static final int kOutputMotor = 50;
    public static final int kCANdi = 52;
    //Current Limits
    public static final double kSupplyCurrentLimit = 20;
    //Speeds
    public static final double kIntakeSpeed_ForIntake = -0.2;
    public static final double kIntakeSpeed_ForOutput = 0.7;
  }
  public static class ClimberConstants{
    //Devices
    public static final int kClimberMotor = 40;
    //Positions
    public static final double kInitialRotation = 0; //TODO: Find this!
    public static final double kEndRotation = 0; //TODO: Find this!
  }
}
