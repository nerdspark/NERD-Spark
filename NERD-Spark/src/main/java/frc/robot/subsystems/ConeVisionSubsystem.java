// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConeVisionSubsystem extends SubsystemBase {

  public static ConeVisionSubsystem instance;
  private PhotonCamera coneCamera;
  private double yawVal = 0;
  private double pitchVal = 0;
  private double skewValue = 0;
  private double areaVal = 0;
  private boolean hasTarget = false;

  /** Creates a new ExampleSubsystem. */
  public ConeVisionSubsystem( PhotonCamera coneCamera) {

    this.coneCamera =coneCamera ;


  }

  public void  setPipeLineIndex(int pipeLineIndex){

    this.coneCamera.setPipelineIndex(pipeLineIndex);

  }
  public static ConeVisionSubsystem getInstance (PhotonCamera coneCamera){

    if (instance == null){
      instance = new ConeVisionSubsystem(coneCamera);
    }
    return instance;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          SmartDashboard.putNumber("Running ConeVisionSubsystem", 9999999);
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    var result = this.coneCamera.getLatestResult();

    if (result.hasTargets()){
      this.yawVal = result.getBestTarget().getYaw();
      this.pitchVal = result.getBestTarget().getPitch();
      this.skewValue = result.getBestTarget().getSkew();
      this.areaVal = result.getBestTarget().getArea();
      this.hasTarget = true;
    }else{
      this.hasTarget = false;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getYawVal(){
    return this.yawVal;
  }
  public double getPitchVal(){
    return this.pitchVal;
  }
  public double getAreVal(){
    return this.areaVal;
  }
  public double getSkewVal(){
    return this.skewValue;
  }
  public boolean hasTargets(){
    return this.hasTarget;
  }

  public List<PhotonTrackedTarget> getTargets(){
    List<PhotonTrackedTarget> targets = null;
    var results = this.coneCamera.getLatestResult();
    if(results.hasTargets()){
      targets = results.getTargets();
    }
  
    return 
    
    targets;
  }

  public double getRange(){

    double range = PhotonUtils.calculateDistanceToTargetMeters(Constants.VisionConstants.CAMERA_HEIGHT_METERS,
     Constants.VisionConstants.CONE_HEIGHT_METERS,
     Constants.VisionConstants.CAMERA_PITCH_RADIANS,
     getPitchVal());

     SmartDashboard.putNumber ("Cone Vision Camera Distance from target", range);

    return range;
  }
}
