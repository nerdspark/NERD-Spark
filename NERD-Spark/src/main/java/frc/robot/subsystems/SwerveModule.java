package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    // private final CANCoder driveEncoder;
    // private final CANCoder turningEncoder;

    private final PIDController turningPidController;

    private final CANCoder CANCoder;
    private final boolean CANCoderReversed;
    private final double CANCoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int CANCoderId, double CANCoderOffset, boolean CANCoderReversed) {

        this.CANCoderOffsetRad = CANCoderOffset;
        this.CANCoderReversed = CANCoderReversed;
        CANCoder = new CANCoder(CANCoderId);

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new TalonFX(turningMotorId);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);


        // driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        // driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        // turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        // turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition()*ModuleConstants.kDriveEncoderRot2Meter;
    }
    
    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition (getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public double getTurningPosition() {
        SmartDashboard.putNumber("pod " + driveMotor.getDeviceID() + " pos", driveMotor.getSelectedSensorPosition()*ModuleConstants.kDriveEncoderRot2Meter);
        return turningMotor.getSelectedSensorPosition() * ModuleConstants.kTurningEncoderRot2Rad;
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity()*ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }

    public double getTurningVelocity() {
        return turningMotor.getSelectedSensorVelocity()*ModuleConstants.kTurningEncoderRPM2RadPerSec*10;
    }

    public double getCANCoderRad() {
        double angle = CANCoder.getAbsolutePosition();
        angle *= 2.0 * Math.PI / 360;
        angle -= CANCoderOffsetRad;
        return angle * (CANCoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turningMotor.setSelectedSensorPosition(getCANCoderRad() / ModuleConstants.kTurningEncoderRot2Rad);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);

        if (Math.abs(state.angle.getRadians() - getTurningPosition()) > 2* Math.PI) {
            state = new SwerveModuleState(state.speedMetersPerSecond, new Rotation2d(((state.angle.getRadians() - getTurningPosition()) % (2*Math.PI)) + getTurningPosition()));
        }
        if ((state.angle.getRadians() - getTurningPosition() > Math.PI)) { //getState().angle.getRadians = getTurningPosition() = current; state.angle.getRadians() = target
            state = new SwerveModuleState(state.speedMetersPerSecond, new Rotation2d(getTurningPosition() - (2*Math.PI)));
        } else if ((state.angle.getRadians() - getState().angle.getRadians()) < -Math.PI) {
            state = new SwerveModuleState(state.speedMetersPerSecond, new Rotation2d(getTurningPosition() + (2*Math.PI)));
        } //short spin for pods

        driveMotor.set(TalonFXControlMode.Velocity, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond * DriveConstants.kFalconMaxSetSpeed);
        turningMotor.set(TalonFXControlMode.Position, state.angle.getRadians() / ModuleConstants.kTurningEncoderRot2Rad);//turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + CANCoder.getDeviceID() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(TalonFXControlMode.PercentOutput, 0);
        turningMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
}