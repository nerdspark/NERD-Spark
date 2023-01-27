// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
	private static final XboxController cont = new XboxController(Constants.controllerPort);

  private static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);

  SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
      () -> driverJoystick.getRawButton(Constants.buttonY)));
      // Configure the button bindings
    configureButtonBindings();

    chooser.addOption("line", loadPathplannerTrajectoryToSwerveController(
      "pathplanner/generatedJSON/line.wpilib.json",
      true));
    chooser.addOption("L", loadPathplannerTrajectoryToSwerveController(
      "pathplanner/generatedJSON/L.wpilib.json",
      true));
    chooser.addOption("square", loadPathplannerTrajectoryToSwerveController(
      "pathplanner/generatedJSON/square.wpilib.json",
      true));
    chooser.addOption("square but angles change", loadPathplannerTrajectoryToSwerveController(
      "pathplanner/generatedJSON/square but angles change.wpilib.json",
      true));
    chooser.addOption("zigzag but angle change", loadPathplannerTrajectoryToSwerveController(
      "pathplanner/generatedJSON/zigzag but angle change.wpilib.json",
      true));
      chooser.addOption("short line", loadPathplannerTrajectoryToSwerveController(
      "pathplanner/generatedJSON/short line.wpilib.json",
      true));
      chooser.addOption("short L", loadPathplannerTrajectoryToSwerveController(
      "pathplanner/generatedJSON/short L.wpilib.json",
      true));
      chooser.addOption("twist", loadPathplannerTrajectoryToSwerveController(
        "pathplanner/generatedJSON/twist.wpilib.json",
        true));
  

    Shuffleboard.getTab("Autonomous").add(chooser);
    
  }

  /**
   * @param filename
   * @param resetOdomtry
   * @return
   */
  public Command loadPathplannerTrajectoryToSwerveController(String filename, boolean resetOdomtry) { 
    Trajectory trajectory;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException exception) {
      DriverStation.reportError("Unable to open trajectory" + filename, exception.getStackTrace());
      System.out.println("Unable to read from file " + filename);
      return new InstantCommand();
    }

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController);
    PIDController yController = new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, AutoConstants.kIYController);
    // xController.setTolerance(0.03);
    // yController.setTolerance(0.03);
    // PIDController thetaController = new PIDController (AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);
    ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
    // 4. Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            swerveSubsystem::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            swerveSubsystem::setModuleStates,
            swerveSubsystem);


    if (resetOdomtry) {
      return new SequentialCommandGroup(
          new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())), swerveControllerCommand);
    } else {
      return swerveControllerCommand;
    }

  }  
    


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(cont, Constants.buttonA).whileHeld(new ExampleCommand(m_exampleSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return chooser.getSelected();
    // // 1. Create trajectory settings
    // swerveSubsystem.resetOdometry(new Pose2d());
    // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    //         AutoConstants.kMaxSpeedMetersPerSecond,
    //         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //                 .setKinematics(DriveConstants.kDriveKinematics);

    // // 2. Generate trajectory
    // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         List.of(
    //                 new Translation2d(1, 0)),
    //                 // new Translation2d(0.7, 0)),
    //         new Pose2d(2, 0, Rotation2d.fromDegrees(0)),
    //         trajectoryConfig);

    // // 3. Define PID controllers for tracking trajectory
    // PIDController xController = new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController);
    // PIDController yController = new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, AutoConstants.kIYController);
    // xController.setTolerance(0.03);
    // yController.setTolerance(0.03);
    // ProfiledPIDController thetaController = new ProfiledPIDController(
    //         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // // 4. Construct command to follow trajectory
    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //         trajectory,
    //         swerveSubsystem::getPose,
    //         DriveConstants.kDriveKinematics,
    //         xController,
    //         yController,
    //         thetaController,
    //         swerveSubsystem::setModuleStates,
    //         swerveSubsystem);

    // // 5. Add some init and wrap-up, and return everything
    // return (new SequentialCommandGroup(
    //   new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())), 
    //   swerveControllerCommand, 
    //   new InstantCommand(() -> swerveSubsystem.stopModules())));
  }
}
