package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_TO_ROBOT;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;

public class PoseEstimatorSubSystem extends SubsystemBase {

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.1);

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, 0.9);

  private final SwerveSubsystem drivetrainSubsystem;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field2d = new Field2d();
  private final PhotonPoseEstimator photonPoseEstimator;

  private double previousPipelineTimestamp = 0;
  private OriginPosition originPosition = OriginPosition.kBlueAllianceWallRightSide;
  private boolean sawTag = false;

  public PoseEstimatorSubSystem(PhotonCamera photonCamera, SwerveSubsystem drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    PhotonPoseEstimator photonPoseEstimator;

    final AprilTag tag1 = new AprilTag(0,FieldConstants.aprilTags3D.get(1));
     final AprilTag tag2 = new AprilTag(0,FieldConstants.aprilTags3D.get(2));
     final AprilTag tag3 = new AprilTag(0,FieldConstants.aprilTags3D.get(3));
     final AprilTag tag4 = new AprilTag(0,FieldConstants.aprilTags3D.get(4));
     final AprilTag tag5 = new AprilTag(0,FieldConstants.aprilTags3D.get(5));
     final AprilTag tag6 = new AprilTag(0,FieldConstants.aprilTags3D.get(6));
     final AprilTag tag7 = new AprilTag(0,FieldConstants.aprilTags3D.get(7));
     final AprilTag tag8 = new AprilTag(0,FieldConstants.aprilTags3D.get(8));
     final ArrayList<AprilTag> atList =  new ArrayList<AprilTag>();
     atList.add(tag1);
     atList.add(tag2);
     atList.add(tag3);
     atList.add(tag4);
     atList.add(tag5);
     atList.add(tag6);
     atList.add(tag7);
     atList.add(tag8);    
    

    try {
      var layout = new AprilTagFieldLayout(atList, FieldConstants.fieldLength, FieldConstants.fieldWidth);
      layout.setOrigin(originPosition);
      photonPoseEstimator =
          new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP, photonCamera, APRILTAG_CAMERA_TO_ROBOT);
    } catch(Exception e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;
    }
    this.photonPoseEstimator = photonPoseEstimator;

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    poseEstimator =  new SwerveDrivePoseEstimator(
      Constants.DriveConstants.kDriveKinematics,
        drivetrainSubsystem.getRotation2d(),
        drivetrainSubsystem.getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);
    
    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
  }

  /**
   * Sets the alliance. This is used to configure the origin of the AprilTag map
   * @param alliance alliance
   */
  public void setAlliance(Alliance alliance) {
    var fieldTags = photonPoseEstimator.getFieldTags();
    boolean allianceChanged = false;
    switch(alliance) {
      case Blue:
        fieldTags.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        allianceChanged = (originPosition == OriginPosition.kRedAllianceWallRightSide);
        originPosition = OriginPosition.kBlueAllianceWallRightSide;
        break;
      case Red:
        fieldTags.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        allianceChanged = (originPosition == OriginPosition.kBlueAllianceWallRightSide);
        originPosition = OriginPosition.kRedAllianceWallRightSide;
        break;
      default:
        // No valid alliance data. Nothing we can do about it
    }
    if (allianceChanged && sawTag) {
      // The alliance changed, which changes the coordinate system.
      // Since a tag was seen, and the tags are all relative to the coordinate system, the estimated pose
      // needs to be transformed to the new coordinate system.
      var newPose = flipAlliance(poseEstimator.getEstimatedPosition());
      setCurrentPose(newPose);
    }
  }

  @Override
  public void periodic() {
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(
        drivetrainSubsystem.getRotation2d(),
        drivetrainSubsystem.getModulePositions());
    
    if (photonPoseEstimator != null) {
      // Update pose estimator with the best visible target
      photonPoseEstimator.update().ifPresent(estimatedRobotPose -> {
        sawTag = true;
        var estimatedPose = estimatedRobotPose.estimatedPose;
        // Make sure we have a new measurement, and that it's on the field
        if (estimatedRobotPose.timestampSeconds != previousPipelineTimestamp
            && estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FieldConstants.fieldLength
            && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FieldConstants.fieldWidth) {
          previousPipelineTimestamp = estimatedRobotPose.timestampSeconds;
          poseEstimator.addVisionMeasurement(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
        }
      });
    }

    Pose2d dashboardPose = getCurrentPose();
    if (originPosition == OriginPosition.kRedAllianceWallRightSide) {
      // Flip the pose when red, since the dashboard field photo cannot be rotated
      dashboardPose = flipAlliance(dashboardPose);
    }
    field2d.setRobotPose(dashboardPose);
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
      drivetrainSubsystem.getRotation2d(),
      drivetrainSubsystem.getModulePositions(),
      newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  /**
   * Transforms a pose to the opposite alliance's coordinate system. (0,0) is always on the right corner of your
   * alliance wall, so for 2023, the field elements are at different coordinates for each alliance.
   * @param poseToFlip pose to transform to the other alliance
   * @return pose relative to the other alliance's coordinate system
   */
  private Pose2d flipAlliance(Pose2d poseToFlip) {
    return poseToFlip.relativeTo(new Pose2d(
      new Translation2d(FieldConstants.fieldLength, FieldConstants.fieldWidth),
      new Rotation2d(Math.PI)));
  }

}
