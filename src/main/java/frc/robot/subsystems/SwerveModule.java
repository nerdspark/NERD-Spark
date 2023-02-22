package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    // private final CANCoder driveEncoder;
    // private final CANCoder turningEncoder;

    // private final PIDController turningPidController;

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

        // turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        // turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }
    public SwerveModulePosition getSwerveModulePosition() {
        // SmartDashboard.putNumber("pod " + CANCoder.getDeviceID() + "CANCoder pos", CANCoder.getAbsolutePosition());
        return new SwerveModulePosition (getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public double getDrivePosition() {
        SmartDashboard.putNumber("pod " + driveMotor.getDeviceID() + " drive pos", driveMotor.getSelectedSensorPosition()*ModuleConstants.kDriveTicks2Meters);
        return driveMotor.getSelectedSensorPosition()*ModuleConstants.kDriveTicks2Meters;
    }
    public double getTurningPosition() {
        SmartDashboard.putNumber("pod " + turningMotor.getDeviceID() + " turn pos", turningMotor.getSelectedSensorPosition() * ModuleConstants.kTurnTicks2Radians);
        return turningMotor.getSelectedSensorPosition() * ModuleConstants.kTurnTicks2Radians;
    }
    public double getDriveVelocity() {
        SmartDashboard.putNumber("pod " + driveMotor.getDeviceID() + " drive vel", driveMotor.getSelectedSensorVelocity()*ModuleConstants.kDriveTicks2MeterPerSecond);
        return driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveTicks2MeterPerSecond;
    }
    public double getTurningVelocity() {
        SmartDashboard.putNumber("pod " + turningMotor.getDeviceID() + " turn vel", turningMotor.getSelectedSensorVelocity()*ModuleConstants.kTurnTicks2RadiansPerSecond);
        return turningMotor.getSelectedSensorVelocity()*ModuleConstants.kTurnTicks2RadiansPerSecond;
    }

    public double getPIDTurningMotor() {
        SmartDashboard.putNumber("PID output", turningMotor.getMotorOutputPercent());
        return turningMotor.getMotorOutputPercent();
    }

    // public double getPIDSetpoint() {
    //     SmartDashboard.putNumber("PID setpoint", turningMotor.getClosedLoopTarget());
    //     return turningMotor.getClosedLoopTarget();
    // }

    public double getDriveAcceleration() {
        return driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveTicks2MeterPerSecond;
    }

    public double getTurningAcceleration() {
        return turningMotor.getSelectedSensorVelocity() * ModuleConstants.kTurnTicks2RadiansPerSecond;
    } 

    public void outputStatsSmartDashboard(){
        // SmartDashboard.putNumber("pod " + driveMotor.getDeviceID() + " pos", CANCoder.getAbsolutePosition());
        // SmartDashboard.putNumber("pod " + driveMotor.getDeviceID() + " vel", CANCoder.getVelocity());
        // SmartDashboard.putNumber("Field to Robot Angle", getCANCoderRad());
        // SmartDashboard.putNumber("X", getDrivePosition());;
        // SmartDashboard.putNumber("Y", getTurningPosition());
        // SmartDashboard.putNumber("X Velocity", getDriveVelocity());
        // SmartDashboard.putNumber("Y Velocity", getTurningVelocity());
        // //accelerometer in g
        // SmartDashboard.putNumber("X Acceleration", getDriveAcceleration());
        // //gyro in degrees per second
        // SmartDashboard.putNumber("Y Acceleration", getTurningAcceleration());
        // //encoder, distance and speed
        // SmartDashboard.putNumber("Distance", getDrivePosition());
        // SmartDashboard.putNumber("Speed", getDriveVelocity());
        // SmartDashboard.putNumber("Gyro", getTurningPosition());
        // //voltage
        // SmartDashboard.putNumber("drive volt", driveMotor.getMotorOutputVoltage());
        // SmartDashboard.putNumber("turn volt", turningMotor.getMotorOutputVoltage());
        // //PID
        // SmartDashboard.putNumber("P", DriveConstants.kPTurningMotor);
        // SmartDashboard.putNumber("I", DriveConstants.kITurningMotor);
        // SmartDashboard.putNumber("D", DriveConstants.kDTurningMotor);
        // SmartDashboard.putNumber("PID output", getPIDTurningMotor());
        // SmartDashboard.putNumber("PID setpoint", getPIDSetpoint());

    }

    public double getCANCoderRad() {
        double angle = CANCoder.getAbsolutePosition();
        SmartDashboard.putNumber("pod " + CANCoder.getDeviceID() + "CANCoder pos", angle);
        angle *= 2.0 * Math.PI / 360;
        angle -= CANCoderOffsetRad;
        return angle * (CANCoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turningMotor.setSelectedSensorPosition(getCANCoderRad() / ModuleConstants.kTurnTicks2Radians);
        turningMotor.config_kP(1, DriveConstants.kPTurningMotor);
        turningMotor.config_kI(1, DriveConstants.kITurningMotor);
        turningMotor.config_kD(1, DriveConstants.kDTurningMotor);
        turningMotor.selectProfileSlot(0, 0);
        // driveMotor.config_kP(1, DriveConstants.kPDriveMotor);
        // driveMotor.config_kI(1, DriveConstants.kIDriveMotor);
        // driveMotor.config_kD(1, DriveConstants.kDDriveMotor);
        // driveMotor.config_kF(1, DriveConstants.kFDriveMotor);
        driveMotor.selectProfileSlot(0, 0);
        // turningMotor.selectProfileSlot(0, 0);
        driveMotor.configClosedloopRamp(DriveConstants.kRampRateDriveMotor);
        turningMotor.configClosedloopRamp(DriveConstants.kRampRateTurningMotor);
    }

    public void setGains() {
        turningMotor.config_kP(1, DriveConstants.kPTurningMotor);
        turningMotor.config_kI(1, DriveConstants.kITurningMotor);
        turningMotor.config_kD(1, DriveConstants.kDTurningMotor);
        turningMotor.selectProfileSlot(0, 0);
        // driveMotor.config_kP(1, DriveConstants.kPDriveMotor);
        // driveMotor.config_kI(1, DriveConstants.kIDriveMotor);
        // driveMotor.config_kD(1, DriveConstants.kDDriveMotor);
        // driveMotor.config_kF(1, DriveConstants.kFDriveMotor);
        driveMotor.selectProfileSlot(0, 0);
        // turningMotor.selectProfileSlot(0, 0);
        driveMotor.configClosedloopRamp(DriveConstants.kRampRateDriveMotor);
        turningMotor.configClosedloopRamp(DriveConstants.kRampRateTurningMotor);
        
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        // state = SwerveModuleState.optimize(state, getState().angle);

        double target = state.angle.getRadians();
        double current = getTurningPosition();
        if (Math.abs(target - current) > (2* Math.PI)) {
            target = (target - current) % (2*Math.PI) + current;
        }
        if ((target - current > Math.PI)) { //getState().angle.getRadians = getTurningPosition() = current; state.angle.getRadians() = target
            target -= (2*Math.PI);
        } else if ((target - current) < -Math.PI) {
            target += (2*Math.PI);
        }
        boolean backward = false;
        if ((target - current) > Math.PI/2) {
            target -= (Math.PI);
            backward = true;
        } else if ((target - current) < -Math.PI/2) {
            target += (Math.PI);
            backward = true;
        } 
            //short spin for pods
        state = new SwerveModuleState(state.speedMetersPerSecond, new Rotation2d(target));//(Math.abs(state.speedMetersPerSecond)/state.speedMetersPerSecond)*Math.abs(Math.pow(state.speedMetersPerSecond, OIConstants.driverPower)), new Rotation2d(target));

        SmartDashboard.putNumber("driveMotorSet" + driveMotor.getDeviceID(), state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond * DriveConstants.kFalconMaxSetSpeed);
        SmartDashboard.putNumber("turnMotorSet" + turningMotor.getDeviceID(), target);
        if (Math.abs(state.speedMetersPerSecond) > DriveConstants.driveSpeedConvertMode) { 
            SmartDashboard.putString("drive mode", "velocity");
            driveMotor.set(TalonFXControlMode.Velocity, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond * DriveConstants.kFalconMaxSetSpeed * (backward ? -1 : 1));
        } else {
            SmartDashboard.putString("drive mode", "VBUS");
            driveMotor.set(TalonFXControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond * DriveConstants.kFalconMaxSetSpeed * (backward ? -1 : 1) / DriveConstants.kFalconMaxSetSpeed);
        }
        turningMotor.set(TalonFXControlMode.Position, target / ModuleConstants.kTurnTicks2Radians);//turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + CANCoder.getDeviceID() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(TalonFXControlMode.PercentOutput, 0);
        turningMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    
}
