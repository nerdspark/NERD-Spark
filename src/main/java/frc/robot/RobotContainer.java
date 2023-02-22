// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.Auton.ThreeElement;
import frc.robot.commands.Auton.line2meters;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private static final XboxController cont = new XboxController(Constants.controllerPort);

    public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
      
    Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRotXAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRotYAxis),      
      () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx), 
      () -> driverJoystick.getPOV(), 
      () -> driverJoystick.getRawAxis(OIConstants.kDriverLeftTrigger), 
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRightTrigger), 
      () -> driverJoystick.getRawButton(Constants.buttonY), 
      () -> driverJoystick.getRawButton(OIConstants.kDriverCancelTurn), 
      () -> driverJoystick.getRawButton(OIConstants.kDriverTopSpeed)));
      // Configure the button bindings
    configureButtonBindings();


    chooser.setDefaultOption("Line 2 Meters", new line2meters(swerveSubsystem));

    chooser.addOption("Auto Three Element", new ThreeElement(swerveSubsystem));


    Shuffleboard.getTab("Autonomous").add(chooser);
  }



  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  public static SwerveSubsystem getSwerveSubsystem(){
    return swerveSubsystem;
  }
}
