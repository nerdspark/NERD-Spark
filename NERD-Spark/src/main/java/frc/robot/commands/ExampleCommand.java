// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NerdSparkPhotonPoseEstimator;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExampleSubsystem m_subsystem;

  private NerdSparkPhotonPoseEstimator photonPoseEstimator;

  
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable visionTable = inst.getTable("photonvision");
  NetworkTable subTable = visionTable.getSubTable("photonvision");
  BooleanSubscriber targetFoundSub = subTable.getBooleanTopic("hasTarget").subscribe(false);
  DoubleArraySubscriber targetPoseSub = subTable.getDoubleArrayTopic("targetPose").subscribe(new double[] {});

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(ExampleSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    photonPoseEstimator = new NerdSparkPhotonPoseEstimator();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean hasTargetNT = targetFoundSub.get();
    double[] targetPoseNT = targetPoseSub.get();

    SmartDashboard.putBoolean("NT Target Found", hasTargetNT);
    SmartDashboard.putNumber("NT Target Pose X", targetPoseNT[0]);
    SmartDashboard.putNumber("NT Target Pose Y", targetPoseNT[1]);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
