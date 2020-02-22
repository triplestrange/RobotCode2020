/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.SwerveDrive;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  // private final SwerveDrive swerveDrive = new SwerveDrive();
  private final ControlPanelSubsystem controlPanel = new ControlPanelSubsystem();

  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  Joystick m_secondController = new Joystick(OIConstants.kSecondControllerPort);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    // swerveDrive.setDefaultCommand(
    //     // A split-stick arcade command, with forward/backward controlled by the left
    //     // hand, and turning controlled by the right.
    //     new RunCommand(() -> swerveDrive.drive(
    //         m_driverController.getRawAxis(1),
    //         m_driverController.getRawAxis(0),
    //         m_driverController.getRawAxis(4), false), swerveDrive));

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_secondController, Button.kB.value)
         .whenPressed(new InstantCommand(controlPanel::displayCurrentColor, controlPanel));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // Create config for trajectory
  //   TrajectoryConfig config =
  //       new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
  //                            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  //           // Add kinematics to ensure max speed is actually obeyed
  //           .setKinematics(SwerveDriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(
    //         new Translation2d(1, 1),
    //         new Translation2d(2, -1)
    //     ),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config
    // );

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     swerveDrive::getPose, //Functional interface to feed supplier
    //     SwerveDriveConstants.kDriveKinematics,

    //     //Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
    //                               AutoConstants.kThetaControllerConstraints),

    //     swerveDrive::setModuleStates,

    //     swerveDrive

    // );

    // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> swerveDrive.drive(0, 0, 0, false));
//  }
}
