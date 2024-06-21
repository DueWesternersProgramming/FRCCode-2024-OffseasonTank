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

  public static final class DriveConstants{

    public static final int kLeft1MotorPort = 1;
    public static final int kLeft2MotorPort = 4;
    public static final int kRight1MotorPort = 3;
    public static final int kRight2MotorPort = 2;

    public static final double kFastSpeedMultiplier = 0.95;
    public static final double kNormalSpeedMultiplier = 0.7325;
    public static final double kSlowSpeedMultiplier = 0.4;
    public static final double kAutoSpeedMultiplier = 0.65;

    public static final double kChargeForwardBalanceTolerance = -8;
    public static final double kChargeBackwardBalanceTolerance = 10;

    public static final double kChargeForwardInitialStartTolerance = 4;
    public static final double kChargeBackwardInitialStartTolerance = -4;
    public static final double kChargeForwardModifiedStartTolerance = 8;
    public static final double kChargeBackwardModifiedStartTolerance = -6;

    public static final double kMaxAccel = 12;
    public static final int kLeft = -1;
    public static final int kRight = 1;
    public static final double kDriveWidth = 25;

    //public static final double kDefaultP = .00038;
    // public static final double kDefaultI = .0000011;
    // public static final double kDefaultD = .0001;
    public static final double kDefaultP = .00030;
    public static final double kDefaultI = .00000011;
    public static final double kDefaultD = .0001;
    public static final int kEncoderCPR = 1024;
    public static final double kGearRatio = 1/10.75;
    public static final double kWheelDiameterInches = 6;
    public static final double kMaxRPM = 5676;
    public static final double kEncoderDistancePerPulse = (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;
    public static final double kMaxRobotSpeed = (kMaxRPM/60) * kGearRatio * (kWheelDiameterInches * Math.PI);

  }

  public static class ArmConstants {  
    
    public static final double kArmAutoLowPosition = -45;
    public static final double kDownPosition = 0;
    public static final double kScorePosition = -13;
    //public static final double kArmAutoPosition = -90;
    public static final double kArmAutoHighPosition = -210;
    public static final double kArmSpeed = 0.90;

    public static final int kMoveUp = 1;
    public static final int kMoveDown = -1;
    
    public static final int kArmMotorPort = 17;

    public static final double kArmSpeedMultiplier = 0.75;
    
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kAsisstControllerPort = 1;

    public static final double kControllerDeadZone = 0.01;

    public static final int kPDPPort = 16;

  }
}
