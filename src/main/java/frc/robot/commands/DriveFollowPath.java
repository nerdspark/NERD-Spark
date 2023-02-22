// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Timer;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class DriveFollowPath extends CommandBase {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final SwerveSubsystem swerveSubsystem;
//   private final PathPlannerTrajectory trajectory;
//   private final PPHolonomicDriveController ppSwerveController;
//   private final boolean resetOdometry;
//   private final String pathName;
//   private final Timer timer = new Timer();
  

// public DriveFollowPath(String pathName){
//   this(pathName, AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared, true);
// }

// public DriveFollowPath(String pathName, double maxVel, double maxAccel){
//   this(pathName, maxVel, maxAccel, true);
// }

// public DriveFollowPath(String pathName, double maxVel, double maxAccel, boolean resetOdometry){
//   addRequirements(RobotContainer.swerveSubsystem);

//   this.trajectory = PathPlanner.loadPath(pathName, maxVel, maxAccel);

//   PIDController xController = new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController);
//   PIDController yController = new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, AutoConstants.kDYController);
//   PIDController thetaController = new PIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController);
//   thetaController.enableContinuousInput(-Math.PI, Math.PI);
//   this.ppSwerveController = new PPHolonomicDriveController(xController, yController, thetaController);
//   this.resetOdometry = resetOdometry;

//   this.pathName = pathName;

// }


//   // /**
//   //  * Creates a new ExampleCommand.
//   //  *
//   //  * @param subsystem The subsystem used by this command.
//   //  */
//   // public DriveFollowPath(ExampleSubsystem subsystem) {
//   //   m_subsystem = subsystem;
//   //   // Use addRequirements() here to declare subsystem dependencies.
//   //   addRequirements(subsystem);
//   // }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     Pose2d initialPose = trajectory.getInitialHolonomicPose();
//     if(resetOdometry) RobotContainer.swerveSubsystem.resetOdometry(new Pose2d(initialPose.getTranslation(), RobotContainer.swerveSubsystem.getRotation2d()));

//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     if (resetOdometry) {
//       return new SequentialCommandGroup(
//           new InstantCommand(() -> RobotContainer.swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
//           ppSwerveControllerCommand);
//     } else {
//       return ppSwerveControllerCommand;
//     }


//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     RobotContainer.swerveSubsystem.stopModules();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
}
