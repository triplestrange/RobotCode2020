/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SwerveControllerCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    public static SwerveDrive swerveDrive = new SwerveDrive();
    private final Intake intake = new Intake();
    private final Conveyor zoom = new Conveyor();
    public final static Shooter shooter = new Shooter();
    private final Climb climb = new Climb();
    public final Vision vision = new Vision();
    private final Turret spinny = new Turret(vision);

    // The driver's controller
    public static Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
    public static Joystick m_operatorController = new Joystick(1);

    public static ProfiledPIDController theta = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
            AutoConstants.kThetaControllerConstraints);

    // Network Tables for Vision
    public NetworkTableEntry yaw;
    public NetworkTableEntry isDriverMode;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Gets the default instance of NetworkTables
        NetworkTableInstance table = NetworkTableInstance.getDefault();

        // Gets the MyCamName table under the chamelon-vision table
        // MyCamName will vary depending on the name of your camera
        NetworkTable cameraTable = table.getTable("chameleon-vision").getSubTable("Shooter");

        // Gets the yaw to the target from the cameraTable
        yaw = cameraTable.getEntry("yaw");

        // Gets the driveMode boolean from the cameraTable
        isDriverMode = cameraTable.getEntry("driver_mode");

        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        swerveDrive.setDefaultCommand(

                new RunCommand(() -> swerveDrive.drive(
                        -m_driverController.getRawAxis(1) * Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond,
                        -m_driverController.getRawAxis(0) * Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond,
                        -m_driverController.getRawAxis(4) * (Math.PI), true), swerveDrive));

        zoom.setDefaultCommand(new RunCommand(zoom::autoIndex, zoom));
        intake.setDefaultCommand(new RunCommand(
                () -> intake.runWheels(m_operatorController.getRawAxis(2), m_operatorController.getRawAxis(3)),
                intake));

        spinny.setDefaultCommand(new RunCommand(
                () -> spinny.spin(m_driverController.getRawAxis(2), m_driverController.getRawAxis(3)),
                spinny));

        // vision.setDefaultCommand(new RunCommand(vision::runVision, vision));
    }

    /**
     * 
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(m_operatorController, 1).whenPressed(new InstantCommand(intake::extend, intake))
                .whenReleased(intake::retract, intake);
        new JoystickButton(m_operatorController, 6).whileHeld(new InstantCommand(shooter::runShooter, shooter))
                .whenReleased(shooter::stopShooter, shooter);
        new JoystickButton(m_operatorController, 9).whileHeld(new RunCommand(() -> shooter.runHood(0), shooter));
        new JoystickButton(m_operatorController, 10).whileHeld(new RunCommand(() -> shooter.runHood(1), shooter));
        new JoystickButton(m_operatorController, 6)
                .whenPressed(new RunCommand(() -> zoom.feedShooter(0.8, shooter.atSpeed()), zoom))
                .whenReleased(new RunCommand(zoom::autoIndex, zoom));
        new JoystickButton(m_operatorController, 5).whenPressed(new RunCommand(() -> zoom.manualControl(-1), zoom))
                .whenReleased(new RunCommand(zoom::autoIndex, zoom));
        // new JoystickButton(m_operatorController, 8).whileHeld(new InstantCommand(vision::runVision, vision))
        //         .whenReleased(vision::filler, vision); // not sure what to put as method here
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(.75,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(SwerveDriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d((Math.PI) / 2)),
        List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(-4, 2.3, new Rotation2d((Math.PI) / 2)), config);




        SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(exampleTrajectory,
                (1.875), swerveDrive::getPose, // Functional interface to feed supplier
                SwerveDriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0), theta,

                swerveDrive::setModuleStates,

                swerveDrive

        );

        Command shootCommand = new InstantCommand(() -> shooter.setHood(0.5), shooter)
                                .andThen(shooter::runShooter, shooter)
                                .andThen(new RunCommand(() -> zoom.feedShooter(0.75, shooter.atSpeed()), zoom))
                                .withTimeout(15).andThen(new InstantCommand(shooter::stopShooter, shooter));

        return swerveControllerCommand1.andThen(() -> swerveDrive.drive(0, 0, 0, false)).andThen(shootCommand);

    }

}
