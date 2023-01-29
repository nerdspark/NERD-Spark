// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.annotation.Target;
import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AprTagCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExampleSubsystem m_subsystem;
  public PhotonCamera photonCamera;
  private int tagToChase = 0;
  private boolean aprTagFound = false;
  private Pose2d goalPose = null;
  private PhotonTrackedTarget lastTarget;
  int count;
  private static final Transform2d TAG_TO_GOAL = new Transform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180.0));


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AprTagCommand(PhotonCamera photonCamera, ExampleSubsystem subsystem, int tagToChase) {
    m_subsystem = subsystem;
    SmartDashboard.putNumber("AprTagPipeLine Constructor", tagToChase);
    // Use addRequirements() here to declare subsystem dependencies.
    this.tagToChase = tagToChase;
    this.photonCamera = photonCamera;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goalPose = null;
    lastTarget = null;
    SmartDashboard.putNumber("AprTagPipeLine INIT", this.tagToChase);  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

     SmartDashboard.putNumber( photonCamera.getName(),count++);

    var robotPose = new Pose2d(0,0,new Rotation2d(0)); // Dummy Pose
    var photonRes = photonCamera.getLatestResult();
    if (photonRes.hasTargets()) {
      SmartDashboard.putNumber("AprTagPipeLine TargetFound", this.tagToChase);
      // Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == this.tagToChase)
          .findFirst();
          
      if (targetOpt.isPresent()) {

        this.aprTagFound = true;

        var target = targetOpt.get();
        if (!target.equals(lastTarget)) {
          // This is new target data, so recalculate the goal
          lastTarget = target;

          
          double X = target.getBestCameraToTarget().getX();

          double Y = target.getBestCameraToTarget().getY();

          double yaw = target.getYaw();

          SmartDashboard.putNumber("AprTagPipeLine Target getFiducialId", target.getFiducialId());
          SmartDashboard.putNumber("AprTagPipeLine Target X", X);
          SmartDashboard.putNumber("AprTagPipeLine Target Y", Y);
          SmartDashboard.putNumber("AprTagPipeLine Target Yaw", yaw);


          // Get the transformation from the camera to the tag (in 2d)
          var camToTarget = target.getBestCameraToTarget();
          var transform = new Transform2d(
            camToTarget.getTranslation().toTranslation2d(),
            camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));
            
            // Transform the robot's pose to find the tag's pose
            var cameraPose = robotPose.transformBy(Constants.VisionConstants.CAMERA_TO_ROBOT.inverse());
            Pose2d targetPose = cameraPose.transformBy(transform);
            
            // Transform the tag's pose to set our goal
            goalPose = targetPose.transformBy(TAG_TO_GOAL);
        }

        if (null != goalPose) {

         // Drive
          SmartDashboard.putNumber("AprTagPipeLine goalPoseX", goalPose.getX());
          SmartDashboard.putNumber("AprTagPipeLine goalPoseY", goalPose.getY());
          SmartDashboard.putNumber("AprTagPipeLine goalPoseAngle", goalPose.getRotation().getDegrees());
        }
      }
  }
  else{
    SmartDashboard.putNumber("AprTagPipeLine No targets found Appa", count++);
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.aprTagFound;

  }
}
