package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningTargX, turningTargY;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final Supplier<Integer> DPAD;
    private final Supplier<Double> leftTrigger;
    private final Supplier<Double> rightTrigger;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final PIDController targetTurnController = new PIDController(DriveConstants.kPTargetTurning, DriveConstants.kITargetTurning, DriveConstants.kDTargetTurning);

    private double targetAngle;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningTargX, Supplier<Double> turningTargY,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Integer> DPAD, Supplier<Double> leftTrigger, Supplier<Double> rightTrigger) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningTargX = turningTargX;
        this.turningTargY = turningTargY;
        this.DPAD = DPAD;
        this.rightTrigger = rightTrigger;
        this.leftTrigger = leftTrigger;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        // swerveSubsystem.setGains();
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = (Math.abs(xSpdFunction.get()*xSpdFunction.get())*xSpdFunction.get()*OIConstants.driverMultiplier);
        double ySpeed = (Math.abs(ySpdFunction.get()*ySpdFunction.get())*ySpdFunction.get()*OIConstants.driverMultiplier);
        double currentAngle = swerveSubsystem.getHeading()*Math.PI/180;
        if (DPAD.get() != -1) {
            targetAngle =  (DPAD.get() * Math.PI / 180d);
        } else 
        if ((leftTrigger.get() > OIConstants.triggerDeadband) || (rightTrigger.get() > OIConstants.triggerDeadband)) {
            targetAngle += ((rightTrigger.get() - leftTrigger.get()) * OIConstants.triggerMultiplier);
        } else 
        if ((turningTargX.get() * turningTargX.get()) + (turningTargY.get() * turningTargY.get()) > (OIConstants.kDeadbandSteer * OIConstants.kDeadbandSteer)) {
            targetAngle = -Math.atan2(-turningTargX.get(), -turningTargY.get());
            // targetAngle = ((targetAngle - currentAngle) % (2 * Math.PI)) + currentAngle;
            // if ((targetAngle - currentAngle) > Math.PI) {
            //     targetAngle -= 2 * Math.PI;
            // } else if ((targetAngle - currentAngle) < -Math.PI) {
            //     targetAngle += 2 * Math.PI;
            // }
        }
        targetTurnController.enableContinuousInput(-Math.PI, Math.PI);
        double turningSpeed = targetTurnController.calculate(currentAngle, targetAngle) ;
        if ((Math.abs(targetAngle - swerveSubsystem.getHeading()))<DriveConstants.kTargetTurningDeadband) {
            turningSpeed = 0;
        }
        // 2. Apply deadband
        
        if ((xSpeed * xSpeed) + (ySpeed * ySpeed) < (OIConstants.kDeadbandDrive * OIConstants.kDeadbandDrive)) {//Math.abs(xSpeed) < OIConstants.kDeadbandDrive && Math.abs(ySpeed) < OIConstants.kDeadbandDrive) {
            xSpeed = 0;
            ySpeed = 0;
        }

        // turningSpeed = Math.abs(turningTargX.get()) > OIConstants.kDeadbandSteer || Math.abs(turningTargY.get()) > OIConstants.kDeadbandSteer ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

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

        // 5. Convert chassis speeds to individuaDriveConstantsl module states
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
}