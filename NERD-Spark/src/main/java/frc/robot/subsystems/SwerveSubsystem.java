package frc.robot.subsystems;


import com.ctre.phoenix.sensors.Pigeon2;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

//For Vision

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveCANCoderPort,
            DriveConstants.kFrontLeftDriveCANCoderOffsetRad,
            DriveConstants.kFrontLeftDriveCANCoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveCANCoderPort,
            DriveConstants.kFrontRightDriveCANCoderOffsetRad,
            DriveConstants.kFrontRightDriveCANCoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveCANCoderPort,
            DriveConstants.kBackLeftDriveCANCoderOffsetRad,
            DriveConstants.kBackLeftDriveCANCoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveCANCoderPort,
            DriveConstants.kBackRightDriveCANCoderOffsetRad,
            DriveConstants.kBackRightDriveCANCoderReversed);

    private final Pigeon2 gyro = new Pigeon2(Constants.pigeonPort);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), new SwerveModulePosition[] {
                frontLeft.getSwerveModulePosition(), 
                frontRight.getSwerveModulePosition(),
                backLeft.getSwerveModulePosition(),
                backRight.getSwerveModulePosition()
            });

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.setYaw(0);
    }

    public double getHeading() {
        return Math.IEEEremainder(-gyro.getYaw(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
            new SwerveModulePosition(), 
            new SwerveModulePosition(), 
            new SwerveModulePosition(), 
            new SwerveModulePosition()
    }, pose);
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), new SwerveModulePosition[] {
                frontLeft.getSwerveModulePosition(), 
                frontRight.getSwerveModulePosition(),
                backLeft.getSwerveModulePosition(),
                backRight.getSwerveModulePosition()
        });
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location (broken)", getPose().getTranslation().toString());
        SmartDashboard.putNumber("X pos", odometer.getPoseMeters().getX());
        SmartDashboard.putNumber("Y pos", odometer.getPoseMeters().getY());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setGains() {
        frontLeft.setGains();
        frontRight.setGains();
        backLeft.setGains();
        backRight.setGains();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
 
        if (false)//((Math.abs(desiredStates[0].angle.getRadians() - frontLeft.getTurningPosition())>15*Math.PI/180) 
        // || (Math.abs(desiredStates[1].angle.getRadians() -frontRight.getTurningPosition())>15*Math.PI/180)
        // || (Math.abs(desiredStates[2].angle.getRadians() - backLeft.getTurningPosition())>15*Math.PI/180)
        // || (Math.abs(desiredStates[3].angle.getRadians() - backRight.getTurningPosition())>15*Math.PI/180)))
        {
            desiredStates[0] = new SwerveModuleState(0, desiredStates[0].angle);
            desiredStates[1] = new SwerveModuleState(0, desiredStates[1].angle);
            desiredStates[2] = new SwerveModuleState(0, desiredStates[2].angle);
            desiredStates[3] = new SwerveModuleState(0, desiredStates[3].angle);
        }

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
            
        
    }

    //For Vision

    public Rotation2d getGyroscopeRotation() {
        // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        // return Rotation2d.fromDegrees(360.0 - navx.getYaw());
        return Rotation2d.fromDegrees(360.0 - gyro.getYaw());
           
    }

    public SwerveModulePosition[] getServeModulePositions(){

        return new SwerveModulePosition[]{
            
            frontLeft.getSwerveModulePosition(), 
            frontRight.getSwerveModulePosition(),
            backLeft.getSwerveModulePosition(),
            backRight.getSwerveModulePosition()
        };
    } 


  }