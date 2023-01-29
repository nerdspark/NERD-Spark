// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExampleSubsystem extends SubsystemBase {

  CANCoder sensor1;

  TalonFX motor1;
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    sensor1 = new CANCoder(Constants.sensor1ID);
    motor1 = new TalonFX(Constants.motorr1ID);
  }

  @Override
  public void periodic() {
   // SmartDashboard.putNumber("sensor position", sensor1.getAbsolutePosition());
  }

  public void runMotor(double power) {
    motor1.set(TalonFXControlMode.PercentOutput, power);
  }

  // public void printSensor() {
    // SmartDashboard.putNumber("sensor position", sensor1.getAbsolutePosition());
  // }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void resetSensor() {
    sensor1.setPosition(0);
  }

  public double returnSensorOutput() {
    return sensor1.getAbsolutePosition();
  }

}
