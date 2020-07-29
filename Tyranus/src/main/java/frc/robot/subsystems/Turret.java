/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

public class Turret extends SubsystemBase {

  // transfer to robot container
  private static final int motor = 10;
  private final CANSparkMax turretMotor;
  private final CANPIDController m_turretPIDController;
  private CANEncoder turretEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, setPoint, rotations;
  private CANDigitalInput m_reverseLimit;
  private DigitalInput limitSwitch;
  public Vision vision;
  public SwerveDrive swerve;
  private PIDController visionController = new PIDController(Constants.Vision.turretKP, Constants.Vision.turretKI, Constants.Vision.turretKD);

  /**
   * Creates a new Turret.
   */
  public Turret(Vision vision, SwerveDrive swerve) {
    this.vision = vision;
    this.swerve = swerve;

    turretMotor = new CANSparkMax(motor, MotorType.kBrushless);
    turretMotor.restoreFactoryDefaults();
    turretMotor.setSmartCurrentLimit(30);
    turretMotor.setIdleMode(IdleMode.kBrake);
    turretMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    turretMotor.setSoftLimit(SoftLimitDirection.kForward, 0);

    turretMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    turretMotor.setSoftLimit(SoftLimitDirection.kReverse, -130);

    turretMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);

    turretMotor.burnFlash();

    turretEncoder = turretMotor.getEncoder();
    turretEncoder.setPosition(0);

    m_turretPIDController = turretMotor.getPIDController();

    // PID coefficients
    kP = 0.1;
    kFF = 1./11000.;
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kMaxOutput = 1;
    kMinOutput = -1;

    // // set PID coefficients
    m_turretPIDController.setP(kP);
    m_turretPIDController.setI(kI);
    m_turretPIDController.setD(kD);
    m_turretPIDController.setIZone(kIz);
    m_turretPIDController.setFF(kFF);
    m_turretPIDController.setOutputRange(kMinOutput, kMaxOutput);

    // m_reverseLimit = turretMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    // m_reverseLimit.enableLimitSwitch(true);
  }

  public void periodic() {
    // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Set Rotations", 0);
    SmartDashboard.putNumber("turretEncoder", turretEncoder.getPosition());
  }

  public void setPosition(double setpoint) {
    m_turretPIDController.setReference(setpoint, ControlType.kPosition);
    SmartDashboard.putNumber("SetPoint", setpoint);
    SmartDashboard.putNumber("ProcessVariable", turretEncoder.getPosition());
  }

  public void stop() {
    turretMotor.set(0);
  }

  public void spin(final double left, final double right) {
        if (left > 0.1)
          turretMotor.set(-left / 2);
        else if (right > 0.1)
          turretMotor.set(right);
     else
     turretMotor.set(0);
  }

  public void visionTurret() {
    double targetYaw = vision.getTargetYaw();
    
    visionController.setSetpoint(0);
    double speed = visionController.calculate(targetYaw);
    turretMotor.set(speed);
  }

  // uses swerve gyro to make sure turret stays pointed at target
  // does name sound weird & should this be in periodic
  public void maintainDirection(Joystick joystick) {
    // gets the heading of the robot from -180 to 180 (should be 0 - intake facing right)
    // double heading = swerve.getHeading();

    // double turretTarget = heading > 0 ? heading - 90 : 0;

    visionController.setSetpoint(0);
  
    // turret should always be facing the target, which is -90 deg
    double speed = visionController.calculate(90);
    
    // makes sure driver isn't trying to override turret
    // third button should be start button
    if (joystick.getRawAxis(2) == 0 && joystick.getRawAxis(3) == 0 && joystick.getRawButton(12) == false) {
      turretMotor.set(speed);
    }

    // not sure if this is the right place to put it, but this is the default command
    // makes joystick rumble if vision target is acquired
    if (vision.hasTargets() == true) {

    }
    
  }

}