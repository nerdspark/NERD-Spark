package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningTargX, turningTargY;
    private final Supplier<Boolean> fieldOrientedFunction, resetGyroButton, cancelTurn, topSpeed;
    private final Supplier<Integer> DPAD;
    private final Supplier<Double> leftTrigger;
    private final Supplier<Double> rightTrigger;
    private final SlewRateLimiter speedLimiter, turningLimiter;
    private final PIDController targetTurnController = new PIDController(DriveConstants.kPTargetTurning, DriveConstants.kITargetTurning, DriveConstants.kDTargetTurning);

    private  double targetAngle;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningTargX, Supplier<Double> turningTargY,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Integer> DPAD, Supplier<Double> leftTrigger, Supplier<Double> rightTrigger, Supplier<Boolean> resetGyroButton, Supplier<Boolean> cancelTurn, Supplier<Boolean> topSpeed) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningTargX = turningTargX;
        this.turningTargY = turningTargY;
        this.DPAD = DPAD;
        this.rightTrigger = rightTrigger;
        this.leftTrigger = leftTrigger;
        this.cancelTurn = cancelTurn;
        this.topSpeed = topSpeed;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.resetGyroButton = resetGyroButton;
        this.speedLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        // swerveSubsystem.setGains();
        zeroHeading();
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double driveAngle = Math.atan2(-ySpdFunction.get(), xSpdFunction.get());
        // double driveSpeed = speedLimiter.calculate(OIConstants.driverMultiplier*Math.pow(Math.abs((ySpdFunction.get()*ySpdFunction.get()) + (xSpdFunction.get()*xSpdFunction.get())), OIConstants.driverPower/2)) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond + OIConstants.driverBaseSpeedMetersPerSecond;
        double driveSpeed = speedLimiter.calculate((topSpeed.get() ? OIConstants.driverTopEXPMultiplier : 
        ((leftTrigger.get() > 0.5) ? OIConstants.driverEXPMultiplier * 0.7 : OIConstants.driverEXPMultiplier))
        *Math.pow(Math.E, 
        Math.abs(
            (Math.abs(ySpdFunction.get()) > Math.abs(xSpdFunction.get()) ? ySpdFunction.get() : xSpdFunction.get())
            *OIConstants.driverEXPJoyMultiplier)))
             * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        double xSpeed = (Math.cos(driveAngle)*driveSpeed);
        double ySpeed = (Math.sin(driveAngle)*driveSpeed);
        // double xSpeed = OIConstants.driverMultiplier*xSpdFunction.get()*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        // double ySpeed = OIConstants.driverMultiplier*ySpdFunction.get()*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        // double driveAngle = Math.atan2(ySpdFunction.get(), xSpdFunction.get());
        // double driveSpeed = speedLimiter.calculate(OIConstants.driverMultiplier*Math.abs((ySpdFunction.get()*ySpdFunction.get()) + (xSpdFunction.get()*xSpdFunction.get()))) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        // double xSpeed = (Math.cos(driveAngle)*driveSpeed);
        // double ySpeed = (Math.sin(driveAngle)*driveSpeed);
        double currentAngle = -swerveSubsystem.getHeading()*Math.PI/180;
        double turningSpeed = 0;
        if (resetGyroButton.get()) {
            zeroHeading();
            swerveSubsystem.resetOdometry(new Pose2d());
        } else 
        if (DPAD.get() != -1) {
            targetAngle =  ((DPAD.get()-90) * Math.PI / 180d);
        } else 
        // if ((leftTrigger.get() > OIConstants.triggerDeadband) || (rightTrigger.get() > OIConstants.triggerDeadband)) {
        //     // targetAngle += ((rightTrigger.get() - leftTrigger.get()) * OIConstants.triggerMultiplier);
        // } else 
        // if ((turningTargX.get() * turningTargX.get()) + (turningTargY.get() * turningTargY.get()) > (OIConstants.kDeadbandSteer * OIConstants.kDeadbandSteer)) {
        //     targetAngle = -Math.atan2(-turningTargX.get(), -turningTargY.get());
        // }

        targetTurnController.enableContinuousInput(-Math.PI, Math.PI);
        turningSpeed = targetTurnController.calculate(currentAngle, targetAngle);
        SmartDashboard.putString("PID turning?", "yes");
        if (((Math.abs(targetAngle - currentAngle) < DriveConstants.kTargetTurningDeadband)/* && !SwerveSubsystem.driveTurning*/) || cancelTurn.get()) {
            turningSpeed = 0;
            SmartDashboard.putString("PID turning?", "disabled");
        }

        if (Math.abs(turningTargX.get()) > OIConstants.kDeadbandSteer) {
            targetAngle = currentAngle;
            turningSpeed = -turningTargX.get() * OIConstants.joystickTurningGain;
            SmartDashboard.putString("PID turning?", "joystickturning");
        } 
        
        // 2. Apply deadband
        
        if (/*Math.abs(xSpdFunction.get() * xSpdFunction.get()) + Math.abs(ySpdFunction.get() * ySpdFunction.get()) < (OIConstants.kDeadbandDrive * OIConstants.kDeadbandDrive)) {*/Math.abs(xSpdFunction.get()) < OIConstants.kDeadbandDrive && Math.abs(ySpdFunction.get()) < OIConstants.kDeadbandDrive) {
            xSpeed = 0;
            ySpeed = 0;
            SmartDashboard.putString("in drive deadband", "yes");
        } else {
            SmartDashboard.putString("in drive deadband", "no");

        }

        // turningSpeed = Math.abs(turningTargX.get()) > OIConstants.kDeadbandSteer || Math.abs(turningTargY.get()) > OIConstants.kDeadbandSteer ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        if ((xSpeed*xSpeed)+(ySpeed*ySpeed) > OIConstants.targetTurnGainScheduleSpeed) {
            SmartDashboard.putString("targetTurnGain", "fastGain");
            turningSpeed = turningSpeed * 1;
        } else {
            SmartDashboard.putString("targetTurnGain", "slowGain");
        }
        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void zeroHeading() {
        swerveSubsystem.zeroHeading();
        targetAngle = 0;
    }
    
}
