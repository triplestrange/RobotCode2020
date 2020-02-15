/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANAnalog;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final CANSparkMax driveMotor;
  private final CANSparkMax steerMotor;

  private final CANEncoder driveEncoder;
  private final CANAnalog steerEncoder;

  private final CANPIDController drivePID;
  private final CANPIDController steerPID;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param steerMotorChannel ID for the steer motor.
   */
  public SwerveModule(int driveMotorChannel, int steerMotorChannel, int[] driveEncoderPorts,
      int[] steerEncoderPorts, boolean driveEncoderReversed, boolean steerEncoderReversed) {

    driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    steerMotor = new CANSparkMax(steerMotorChannel, MotorType.kBrushless);

    driveMotor.restoreFactoryDefaults();
    steerMotor.restoreFactoryDefaults();

    drivePID = driveMotor.getPIDController();
    steerPID = steerMotor.getPIDController();

    driveEncoder = driveMotor.getEncoder();
    steerEncoder = steerMotor.getAnalog(CANAnalog.AnalogMode.kAbsolute);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse);

    // Set whether drive encoder should be reversed or not
    driveEncoder.setInverted(driveEncoderReversed);

    // Set the distance (in this case, angle) per pulse for the steer encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    steerEncoder.setPositionConversionFactor(ModuleConstants.ksteerEncoderDistancePerPulse);
    steerEncoder.setVelocityConversionFactor(ModuleConstants.ksteerEncoderDistancePerPulse);

    // Set whether steer encoder should be reversed or not
    steerEncoder.setInverted(steerEncoderReversed);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(steerEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
  //   // Calculate the drive output from the drive PID controller.
  //   // final var driveOutput = m_drivePIDController.calculate(
  //   //     driveEncoder.getRate(), state.speedMetersPerSecond);

  //   // Calculate the steer motor output from the steer PID controller.
  //   // final var turnOutput = m_steerPIDController.calculate(
  //   //     steerEncoder.get(), state.angle.getRadians()
  //   );

  //   // Calculate the steer motor output from the steer PID controller.
  //   driveMotor.set(driveOutput);
  //   steerMotor.set(turnOutput);

  }

  /**
   * Zeros all the SwerveModule encoders.
   */

  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }
}
