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
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
 
 public class NerdSparkPhotonPoseEstimator
 
 {
     public PhotonCamera photonCamera;
     public RobotPoseEstimator photonPoseEstimator;

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

     
     private ArrayList<AprilTag> atList;
     public NerdSparkPhotonPoseEstimator() {              
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
                         Constants.VisionConstants.cameraName); // Change the name of your camera here to whatever it is in the
         // PhotonVision UI.

         var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(photonCamera, Constants.VisionConstants.robotToCam));
 
         // Create pose estimator
         photonPoseEstimator =
                 new RobotPoseEstimator(
                         atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,camList);
     }
 
     /**
      * @param estimatedRobotPose The current best guess at robot pose
      * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
      *     of the observation. Assumes a planar field and the robot is always firmly on the ground
      */
     public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
         photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
         double currentTime = Timer.getFPGATimestamp();
         Optional<Pair<Pose3d, Double>> result = photonPoseEstimator.update();
         if (result.isPresent()) {
             return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
         } else {
             return new Pair<Pose2d, Double>(null, 0.0);
         }
     }

    //  @Override
    //  public void periodic() {
    //    // Update pose estimator with visible targets
    //    var pipelineResult = photonCamera.getLatestResult();
    //    if (!pipelineResult.equals(previousPipelineResult) && pipelineResult.hasTargets()) {
    //      previousPipelineResult = pipelineResult;
    //      double imageCaptureTime = Timer.getFPGATimestamp() - (pipelineResult.getLatencyMillis() / 1000d);
   
    //      for (PhotonTrackedTarget target : pipelineResult.getTargets()) {
   
    //        var fiducialId = target.getFiducialId();
    //        if (fiducialId >= 0 && fiducialId < atList.size()) {
    //          var targetPose = atList.get(fiducialId).pose.toPose2d();
   
    //          Transform3d camToTarget = target.getBestCameraToTarget();
    //          var transform = new Transform2d(
    //              camToTarget.getTranslation().toTranslation2d(),
    //              camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));
   
    //          Pose2d camPose = targetPose.transformBy(transform.inverse());
   
    //          visionPose = camPose.transformBy(Constants.VisionConstants.CAMERA_TO_ROBOT);
    //          SmartDashboard.putString("Vision pose", String.format("(%.2f, %.2f) %.2f",
    //          visionPose.getTranslation().getX(),
    //          visionPose.getTranslation().getY(),
    //          visionPose.getRotation().getDegrees()));
    //        }
    //      }
    //    }
    // //    Update pose estimator with drivetrain sensors
       

    //  }


     
 }