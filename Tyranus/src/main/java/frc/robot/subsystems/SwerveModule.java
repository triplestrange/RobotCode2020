/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANAnalog;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final CANEncoder driveEncoder;
  private final CANAnalog turningEncoder;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel,
                      int turningMotorChannel,
                      int[] driveEncoderPorts,
                      int[] turningEncoderPorts,
                      boolean driveEncoderReversed,
                      boolean turningEncoderReversed) {

    driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();

    turningEncoder = turningMotor.getAnalog(CANAnalog.AnalogMode.kAbsolute);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse);

    //Set whether drive encoder should be reversed or not
    driveEncoder.setInverted(driveEncoderReversed);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderDistancePerPulse);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderDistancePerPulse);

    //Set whether turning encoder should be reversed or not
    turningEncoder.setInverted(turningEncoderReversed);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turningEncoder.getPosition()));
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

  //   // Calculate the turning motor output from the turning PID controller.
  //   // final var turnOutput = m_turningPIDController.calculate(
  //   //     turningEncoder.get(), state.angle.getRadians()
  //   );

  //   // Calculate the turning motor output from the turning PID controller.
  //   driveMotor.set(driveOutput);
  //   turningMotor.set(turnOutput);
  }

  /**
   * Zeros all the SwerveModule encoders.
   */

  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }
}
