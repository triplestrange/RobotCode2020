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
import com.revrobotics.ControlType;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  // motors
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  // encoders
  private final CANEncoder m_driveEncoder;
  final CANEncoder m_turningEncoder;
  final CANAnalog m_absoluteEncoder;


  // steering pid
  private CANPIDController m_pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  //drive pid
  private CANPIDController m_drivepidController;
  public double dkP, dkI, dkD, dkIz, dkFF, dkMaxOutput, dkMinOutput;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, double encoderCPR, boolean turningEncoderReversed) {

    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveMotor.restoreFactoryDefaults();
    m_turningMotor.restoreFactoryDefaults();

    m_driveEncoder = new CANEncoder(m_driveMotor);
    m_turningEncoder = new CANEncoder(m_turningMotor);
    m_absoluteEncoder = new CANAnalog(m_turningMotor, AnalogMode.kAbsolute);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse);
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse);

    // Set whether drive encoder should be reversed or not

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kSteerEncoderDistancePerPulse);
    m_absoluteEncoder.setPositionConversionFactor(encoderCPR);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    // m_turningPIDController.enableContinuousInput(0, 2*Math.PI);

    // PID coefficients
    kP = 1;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    m_pidController = m_turningMotor.getPIDController();
    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

    // Unused SparkMax DrivePID
    m_drivepidController = m_driveMotor.getPIDController();
    dkP = 0.5;
    dkI = 0;
    dkD = 0;
    dkIz = 0;
    dkFF = 0;
    dkMaxOutput = 1;
    dkMinOutput = -1;
    m_drivepidController.setP(dkP);
    m_drivepidController.setI(dkI);
    m_drivepidController.setD(dkD);
    m_drivepidController.setIZone(dkIz);
    m_drivepidController.setFF(dkFF);
    m_drivepidController.setOutputRange(dkMinOutput, dkMaxOutput);

  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != kP)) {
      m_pidController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      m_pidController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      m_pidController.setD(d);
      kD = d;
    }
    if ((iz != kIz)) {
      m_pidController.setIZone(iz);
      kIz = iz;
    }
    if ((ff != kFF)) {
      m_pidController.setFF(ff);
      kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      m_pidController.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }

    double desiredDrive = state.speedMetersPerSecond;
    double desiredSteering = state.angle.getRadians();
    double currentSteering = m_turningEncoder.getPosition();

    // calculate shortest path to angle with forward drive (error -pi to pi)
    double steeringError = Math.IEEEremainder(desiredSteering - currentSteering, 2 * Math.PI);

    // reverse drive if error is larger than 90 degrees
    if (steeringError > Math.PI / 2) {
      steeringError -= Math.PI;
      desiredDrive *= -1;
    } else if (steeringError < -Math.PI / 2) {
      steeringError += Math.PI;
      desiredDrive *= -1;
    }

    double steeringSetpoint = currentSteering + steeringError;

    // sketchy code to show people it moves
    m_driveMotor.set(desiredDrive);

    // This works nicely, except:
    // >The robot doesn't take the quickest path and then reverse the drive motor
    // >When it hits 360 degrees, it rotates all the way around to 0 (see above)
    m_pidController.setReference(steeringSetpoint, ControlType.kPosition);
    SmartDashboard.putNumber("SetPoint", steeringSetpoint);
    SmartDashboard.putNumber("ProcessVariable", m_turningEncoder.getPosition());
  }

  /**
   * Zeros all the SwerveModule encoders.
   */

  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(Math.PI-m_absoluteEncoder.getPosition());
  }

}