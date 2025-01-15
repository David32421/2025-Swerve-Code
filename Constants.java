// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {

    public static final class ModuleConstants {

        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1/6.75;
        public static final double kTurningMotorGearRatio = 1/(150/7);
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio *Math.PI *kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio*2*Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter/60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad/60;
        public static final double kPTurning = 0.1;

    }
public static final class OIConstants{

    public static double kDeadband = 0.05;
    public static int kDriverControllerPort = 0;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 2;


}
// public static final class DriveConstants {
//     public static final double kTrackWidth = Units.inchesToMeters(21);
//     //distance between right and left wheels
//     public static final double kWheelBase = Units.inchesToMeters(25.5);
//     //distance between front and back wheels
//     public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
//         new Translation2d(kWheelBase/2, -kTrackWidth/2),
//         new Translation2d(kWheelBase/2, kTrackWidth/2),
//         new Translation2d(-kWheelBase/2,-kTrackWidth/2),
//         new Translation2d(-kWheelBase/2, kTrackWidth/2) );

//     public static final int kFrontLeftDriveMotorPort = 8;
//     public static final int kBackLeftDriveMotorPort = 2;
//     public static final int kFrontRightDriveMotorPort = 6;
//     public static final int kBackRightDriveMotorPort = 4;

//     public static final int kFrontLeftTurningMotorPort = 7;
//     public static final int kBackLeftTurningMotorPort = 1;
//     public static final int kFrontRightTurningMotorPort = 3;
//     public static final int kBackRightTurningMotorPort = 9;
//     public static double kPhysicalMaxSpeedMetersPerSecond;
//     public static boolean kFrontLeftTurningEncoderReversed;
//     public static boolean kFrontLeftDriveEncoderReversed;
//     public static int kFrontLeftDriveAbsoluteEncoderPort;
//     public static double kFrontLeftDriveAbsoluteEncoderOffsetRad;
//     public static boolean kFrontLeftDriveAbsoluteEncoderReversed;
//     public static boolean kFrontRightDriveEncoderReversed;
//     public static boolean kFrontRightTurningEncoderReversed;
//     public static double kFrontRightDriveAbsoluteEncoderOffsetRad;
//     public static boolean kFrontRightDriveAbsoluteEncoderReversed;
//     public static int kFrontRightDriveAbsoluteEncoderPort;
//     public static double kTeleDriveMaxAccelerationUnitsPerSecond;
//     public static double kTeleDriveMaxAngularAccelerationUnitsPerSecond;
//     public static double kTeleDriveMaxSpeedMetersPerSecond;
//     public static double kTeleDriveMaxAngularSpeedRadiansPerSecond;
//     public static boolean kBackLeftDriveEncoderReversed;
//     public static int kBackLeftDriveAbsoluteEncoderPort;
//     public static boolean kBackLeftTurningEncoderReversed;
//     public static boolean kBackLeftDriveAbsoluteEncoderReversed;
//     public static double kBackLeftDriveAbsoluteEncoderOffsetRad;
//     public static boolean kBackRightDriveEncoderReversed;
//     public static boolean kBackRightTurningEncoderReversed;
//     public static int kBackRightDriveAbsoluteEncoderPort;
//     public static double kBackRightDriveAbsoluteEncoderOffsetRad;
//     public static boolean kBackRightDriveAbsoluteEncoderReversed;

// }

public static final class DriveConstants {

    public static final double kTrackWidth = Units.inchesToMeters(24.75);
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(24.75);
    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    public static final int kFrontLeftDriveMotorPort = 7;
    public static final int kBackLeftDriveMotorPort = 1;
    public static final int kFrontRightDriveMotorPort = 5;
    public static final int kBackRightDriveMotorPort = 3;

    public static final int kFrontLeftTurningMotorPort = 8;
    public static final int kBackLeftTurningMotorPort = 2;
    public static final int kFrontRightTurningMotorPort = 6;
    public static final int kBackRightTurningMotorPort = 4;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 11;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 14;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 12;
    public static final int kBackRightDriveAbsoluteEncoderPort = 13;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

     public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.5;//-0.254;
     public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -0.4143;//-1.252;
     public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;//-1.816;
     public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.4612;//-4.811;

    //public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
    //public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
    //public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;
    //public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
            kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
}
}

