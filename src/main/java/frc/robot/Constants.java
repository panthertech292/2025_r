// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class ElevatorConstants {
    public static enum ElevatorHeights{DOWN, L3, L4, ALGEE_LOW, ALGEE_HIGH};
    public static final int kLeftElevatorMotor = 20;
    public static final int kRightElevatorMotor = 21;
    public static final int kCANdi = 22;

    public static final double kElevatorGearDiameter = 2.551;
    public static final int kIntEncoderToExtRatio = 64;

    public static final double kElevatorStatorCurrentLimit = 20; //TODO: Tune this

    public static final double kElevatorMaxHeight = 24.176; //this is 3.016 rotations
    public static final double kElevatorMinHeight = 0.25; //TODO: Tune this
    public static final double kElevatorL3Height = 5; //TODO: Tune this
    public static final double kElevatorL4Height = 10; //TODO: Tune this
    public static final double kElevatorLowAlgeeHeight = 0.25; //TODO: Tune this
    public static final double kElevatorHighAlgeeHeight = 0.25; //TODO: Tune this
    public static final double kElevatorEncoderOffset = .809;
  }
  public static class GrabberConstants {
    public static enum GrabberLocations{STOWED,L1, L2, L3, L4, ALGEE_LOW, ALGEE_HIGH, STATION}
    public static final int kRotateMotor = 30;
    public static final int kTranslationMotor = 31; //TODO: Set this motor;
    public static final int kCANdi = 32;

    public static final double kRotateIntEncoderToExtRatio = 16 * (145/18); // (Should be 128.89). 16 is from gearbox, 145/18 is the pulley. 
    public static final double kTranslateIntEncoderToExtRatio = 16; //16 from gearbox
    public static final double kTranslatePulleyDiameter = 0; //TODO: Find this!
    public static final double kRotateEncoderOffset = 0.189;
    public static final double kTranslationEncoderOffset = 0; //TODO: Find this!

    public static final double kRotateStatorCurrentLimit = 20; //TODO: Tune this!
    public static final double kTranslateStatorCurrentLimit = 20; //TODO: Tune this

    public static final double kRotationMaxClockwise = 0.15; //.25 full limit
    public static final double kRotationMinCounterClockwise = -0.15; //-.24 full limit

    public static final double KTranslationMaxLeft = 0; //TODO: Find this!
    public static final double kTranslationMaxRight = 0; //TODO: Find this!
    public static final double kTranslationStowed = 0;

    public static final double kRotationStowed = 0;
    public static final double kRotationL1 = 0; //TODO: Find this!
    public static final double kRotationL2 = 0; //TODO: Find this!
    public static final double kRotationL3 = 0; //TODO: Find this!
    public static final double kRotationL4 = 0; //TODO: Find this!
    public static final double kRotationLowAlgee = 0; //TODO: Find this!
    public static final double kRotationHighAlgee = 0; //TODO: Find this!
  }
  public static class IntakeOutputConstants{
    public static final int kIntakeMotor = 51;
    public static final int kOutputMotor = 50;

    public static final double kStatorCurrentLimit = 20;
  }
}
