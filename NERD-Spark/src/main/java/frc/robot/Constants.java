// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int sensor1ID = 21;
    public static final int motorr1ID = 12;
    public static final int controllerPort = 0;
    public static final int buttonA = 1;
    public static final int pigeonPort = 25;

    public static final int buttonY = 4;


    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);//TODO: figure out right number
        //public static final double kDriveMotorGearRatio = 1d / 2048d;//TODO: figure out right number
        public static final double kDriveMotorGearRatio = 1d / 6.12d;
        //public static final double kTurningMotorGearRatio = 1d / 21.429d / 2048d;//TODO: figure out right number
        public static final double kTurningMotorGearRatio = 1d / (150d / 7d);
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2d * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60d;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60d;
        // public static final double kPTurning = -0.1d;
        // public static final double kTurningEncoderRadians2Ticks = 43008 / 2 / Math.PI;//TODO: figure out right number
        public static final double kDriveEncoderTicks2Rot = 1d / 2048d; //TODO: figure out right number
        // public static final double kDriveTicksPer100ms2RPM = kDriveEncoderTicks2Rot * 600;
        // public static final double kDriveTicks2RevsPerSecond = kDriveEncoderTicks2Rot * 10;
        public static final double kTurningEncoderTicks2Rot = 1d / 2048d;
        public static final double kDriveTicks2Meters = kDriveEncoderRot2Meter * kDriveEncoderTicks2Rot;
        public static final double kDriveTicks2MeterPerSecond = kDriveEncoderTicks2Rot * 10 * kDriveEncoderRot2Meter;
        public static final double kTurnTicks2Radians = kTurningEncoderRot2Rad * kTurningEncoderTicks2Rot;
        public static final double kTurnTicks2RadiansPerSecond = kTurningEncoderTicks2Rot * 10 * kTurningEncoderRot2Rad;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(17);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(17);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 15;
        public static final int kBackLeftDriveMotorPort = 17;
        public static final int kFrontRightDriveMotorPort = 13;
        public static final int kBackRightDriveMotorPort = 11;

        public static final int kFrontLeftTurningMotorPort = 16;
        public static final int kBackLeftTurningMotorPort = 18;
        public static final int kFrontRightTurningMotorPort = 14;
        public static final int kBackRightTurningMotorPort = 12;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final int kFrontLeftDriveCANCoderPort = 23;
        public static final int kBackLeftDriveCANCoderPort = 24;
        public static final int kFrontRightDriveCANCoderPort = 22;
        public static final int kBackRightDriveCANCoderPort = 21;

        public static final boolean kFrontLeftDriveCANCoderReversed = false;
        public static final boolean kBackLeftDriveCANCoderReversed = false;
        public static final boolean kFrontRightDriveCANCoderReversed = false;
        public static final boolean kBackRightDriveCANCoderReversed = false;

        public static final double kFrontLeftDriveCANCoderOffsetRad = 25.1*Math.PI/180;
        public static final double kBackLeftDriveCANCoderOffsetRad = 94.6*Math.PI/180;
        public static final double kFrontRightDriveCANCoderOffsetRad = -56.9*Math.PI/180;
        public static final double kBackRightDriveCANCoderOffsetRad = (173.6-180)*Math.PI/180;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond/1;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 7;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;


        public static final double kFalconMaxSetSpeed = 60000;

        public static final double kPTargetTurning = 0d;
        public static final double kITargetTurning = 0d;
        public static final double kDTargetTurning = 0d;
        public static final double kTargetTurningDeadband = 1;

        public static final double kPTurningMotor = 0.1d;
        public static final double kITurningMotor = 0;
        public static final double kDTurningMotor = 0;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 8;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 0.3d;
        public static final double kPYController = 0.3d;
        public static final double kPThetaController = 2;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 5;
        public static final int kDriverXAxis = 4;
        public static final int kDriverRotXAxis = 0;
        public static final int kDriverRotYAxis = 1;
        public static final int kDriverFieldOrientedButtonIdx = 2;
        public static final int kDriverLeftTrigger = 2;
        public static final int kDriverRightTrigger = 3;

        public static final double kDeadbandSteer = 0.75;
        public static final double kDeadbandDrive = 0.0;

        public static final double driverMultiplier = 0.8;

        public static final double triggerMultiplier = 2;
        public static final double triggerDeadband = 0.1;

    }

    public static class VisionConstants {

        /**
         * Physical location of the camera on the robot, relative to the center of the robot.
         */
        public static final Transform2d CAMERA_TO_ROBOT = 
            new Transform2d(new Translation2d(inchesToMeters(12.75), 0.0), new Rotation2d(0.0));
      }
}
