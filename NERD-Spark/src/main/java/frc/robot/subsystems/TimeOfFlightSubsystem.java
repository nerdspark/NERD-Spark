// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.ThreadPoolExecutor.DiscardOldestPolicy;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TimeOfFlightSubsystem extends SubsystemBase {
com.playingwithfusion.TimeOfFlight timeOfFlight;

  /** Creates a new ExampleSubsystem. */
  public TimeOfFlightSubsystem() {
       this.timeOfFlight = new TimeOfFlight(5);
       timeOfFlight.setRangingMode(RangingMode.Short, 24); 
        
  }

  @Override
  public void periodic() {
   // SmartDashboard.putNumber("sensor position", sensor1.getAbsolutePosition());
  //  SmartDashboard.putNumber("Range", timeOfFlight.getRange());
  }


  // public void printSensor() {
    // SmartDashboard.putNumber("sensor position", sensor1.getAbsolutePosition());
  // }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getRange() {
    return timeOfFlight.getRange();
  }

  public void closeTimeOfFlight() {
    timeOfFlight.close();
  }

 

  

 

}
