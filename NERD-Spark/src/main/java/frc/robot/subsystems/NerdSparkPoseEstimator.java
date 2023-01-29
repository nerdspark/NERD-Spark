/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 
 public class NerdSparkPoseEstimator extends SubsystemBase
 
 {
     private PhotonCamera photonCamera;
     private SwerveSubsystem driveTrainSubsystem; 
     public PhotonPoseEstimator photonPoseEstimator;

     final AprilTag tag1 = new AprilTag(0,FieldConstants.aprilTags3D.get(1));
     final AprilTag tag2 = new AprilTag(0,FieldConstants.aprilTags3D.get(2));
     final AprilTag tag3 = new AprilTag(0,FieldConstants.aprilTags3D.get(3));
     final AprilTag tag4 = new AprilTag(0,FieldConstants.aprilTags3D.get(4));
     final AprilTag tag5 = new AprilTag(0,FieldConstants.aprilTags3D.get(5));
     final AprilTag tag6 = new AprilTag(0,FieldConstants.aprilTags3D.get(6));
     final AprilTag tag7 = new AprilTag(0,FieldConstants.aprilTags3D.get(7));
     final AprilTag tag8 = new AprilTag(0,FieldConstants.aprilTags3D.get(8));
 
     private PhotonPipelineResult previousPipelineResult = null;
     Pose2d visionPose;

     // Note - drivetrain encoders are also used. The Drivetrain class must pass us
    // the relevant readings.

    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your
    // various sensors. Smaller numbers will cause the filter to "trust" the
    // estimate from that particular
    // component more than the others. This in turn means the particualr component
    // will have a stronger
    // influence on the final pose estimate.
    Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    // Matrix<N3, N1> localMeasurementStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));
    Matrix<N3, N1> visionMeasurementStdDevs =
            VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));

     private final SwerveDrivePoseEstimator swerverPoseEstimator;

     private ArrayList<AprilTag> atList;
     public NerdSparkPoseEstimator(SwerveSubsystem driveTrainSubsytem, PhotonCamera photonCamera) {  
        this.driveTrainSubsystem = driveTrainSubsytem;
        this.photonCamera = photonCamera;

        swerverPoseEstimator = new SwerveDrivePoseEstimator(Constants.DriveConstants.kDriveKinematics,
         driveTrainSubsytem.getRotation2d(),driveTrainSubsytem.getModulePositions(), 
         new Pose2d(),stateStdDevs, visionMeasurementStdDevs);

        atList = new ArrayList<AprilTag>();
         atList.add(tag1);
         atList.add(tag2);
         atList.add(tag3);
         atList.add(tag4);
         atList.add(tag5);
         atList.add(tag6);
         atList.add(tag7);
         atList.add(tag8);


         // TODO - once 2023 happens, replace this with just loading the 2023 field arrangement
         AprilTagFieldLayout atfl =
                 new AprilTagFieldLayout(atList, FieldConstants.fieldLength, FieldConstants.fieldWidth);
 
         // Forward Camera
         photonCamera =
                 new PhotonCamera(
                         Constants.VisionConstants.aprTagCameraName); // Change the name of your camera here to whatever it is in the
         // PhotonVision UI.
 
         // Create pose estimator
         photonPoseEstimator =
                 new PhotonPoseEstimator(
                         atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,photonCamera,Constants.VisionConstants.robotToCam);
    }
 
     
     public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

     @Override
     public void periodic() {
       // Update pose estimator with visible targets
       var pipelineResult = photonCamera.getLatestResult();
       if (!pipelineResult.equals(previousPipelineResult) && pipelineResult.hasTargets()) {
         previousPipelineResult = pipelineResult;
         double imageCaptureTime = Timer.getFPGATimestamp() - (pipelineResult.getLatencyMillis() / 1000d);
   
         for (PhotonTrackedTarget target : pipelineResult.getTargets()) {
   
           var fiducialId = target.getFiducialId();
           if (fiducialId >= 0 && fiducialId < atList.size()) {
             var targetPose = atList.get(fiducialId).pose.toPose2d();
   
             Transform3d camToTarget = target.getBestCameraToTarget();
             var transform = new Transform2d(
                 camToTarget.getTranslation().toTranslation2d(),
                 camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));
   
             Pose2d camPose = targetPose.transformBy(transform.inverse());
   
             visionPose = camPose.transformBy(Constants.VisionConstants.CAMERA_TO_ROBOT);
             SmartDashboard.putString("Vision pose", String.format("(%.2f, %.2f) %.2f",
             visionPose.getTranslation().getX(),
             visionPose.getTranslation().getY(),
             visionPose.getRotation().getDegrees()));

             swerverPoseEstimator.addVisionMeasurement(visionPose, imageCaptureTime);
           }
         }
       }
    //    Update pose estimator with drivetrain sensors
       
    swerverPoseEstimator.updateWithTime(Timer.getFPGATimestamp(),
     driveTrainSubsystem.getRotation2d(), driveTrainSubsystem.getModulePositions());
  }

  public Pose2d getCurrentPose(){
    return swerverPoseEstimator.getEstimatedPosition();
  }

  public void setCurrentPose(Pose2d newPose){
    // driveTrainSubsystem.resetGyro();
    swerverPoseEstimator.resetPosition(driveTrainSubsystem.getGyroRotation(), driveTrainSubsystem.getModulePositions(), newPose);
  }
     
 }