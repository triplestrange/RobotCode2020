/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class SwerveDriveConstants {
    public static final int frontLeftDrive = 1;
    public static final int backLeftDrive = 3;
    public static final int frontRightDrive = 2;
    public static final int backRightDrive = 4;

    public static final int frontLeftSteer = 5;
    public static final int backLeftSteer = 7;
    public static final int frontRightSteer = 6;
    public static final int backRightSteer = 8;

    public static final boolean frontLeftSteerEncoderReversed = false;
    public static final boolean backLeftSteerEncoderReversed = false;
    public static final boolean frontRightSteerEncoderReversed = false;
    public static final boolean backRightSteerEncoderReversed = false;

    public static final double kTrackWidth = 0.46355;
    //Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.71755;
    //Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
          new Translation2d(kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = true;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 3;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final double kDriveEncoderCPR = 10.5;
    public static final double kSteerEncoderCPR = 3.3;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kDriveEncoderCPR;

    public static final double ksteerEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kSteerEncoderCPR;

    public static final double kPModulesteerController = 1;

    public static final double kPModuleDriveController = 1;

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;

  }

  public static final class AutoConstants {}
  public static final class Intake {
    public static final int MOTOR = 1;
  }

  public static final class Climb {
    public static final int L_MOTOR = 0;
    public static final int R_MOTOR = 1;
  }

  public static class Controller {
    public final static int A = 1;
    public final static int B = 2;
    public final static int X = 3;
    public final static int Y = 4;
    public final static int LEFT_BUMPER = 5;
    public final static int RIGHT_BUMPER = 6;
    public final static int LEFT_FACE = 7;
    public final static int RIGHT_FACE = 8;
    public final static int JOY_LEFT = 9;
    public final static int JOY_RIGHT = 10;

    public final static int LX = 0;
    public final static int LY = 1;
    public final static int LT = 2;
    public final static int RT = 3;
    public final static int RX = 4;
    public final static int RY = 5;

    public final static int UP = 0;
    public final static int RIGHT = 90;
    public final static int DOWN = 180;
    public final static int LEFT = 270;

}
    

}
