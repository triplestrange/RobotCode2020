/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

public class Turret extends PIDSubsystem {

  //transfer to robot container
  private static final int deviceID = 1;

  private final CANSparkMax turretMotor;
  private final CANPIDController m_turretPIDController;
  private CANEncoder turretEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, rotations;
  private CANDigitalInput m_forwardLimit;
  private CANDigitalInput m_reverseLimit;

  public String kEnable;
  public String kDisable;



   /**
   * Creates a new Turret.
   */
  public Turret() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));

        turretMotor = new CANSparkMax(deviceID, MotorType.kBrushless);
        
        turretMotor.restoreFactoryDefaults();

        m_turretPIDController = turretMotor.getPIDController();
        turretEncoder = turretMotor.getEncoder();
  }

  public void PIDCoefficients() {
    //These are the PID coefficients that need to be placed into the robot container
    // PID coefficients
    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;
  }

  public void setPIDCoefficients() {
    // set PID coefficients
    m_turretPIDController.setP(kP);
    m_turretPIDController.setI(kI);
    m_turretPIDController.setD(kD);
    m_turretPIDController.setIZone(kIz);
    m_turretPIDController.setFF(kFF);
    m_turretPIDController.setOutputRange(kMinOutput, kMaxOutput);
  }
  
  public void displayPIDCoefficients() {
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
  }


  /**
     * A CANDigitalInput object is constructed using the getForwardLimitSwitch() or
     * getReverseLimitSwitch() method on an existing CANSparkMax object, depending
     * on which direction you would like to limit
     * 
     * Limit switches can be configured to one of two polarities:
     *  com.revrobotics.CANDigitalInput.LimitSwitchPolarity.kNormallyOpen
     *  com.revrobotics.CANDigitalInput.LimitSwitchPolarity.kNormallyClosed
     */
  public void limitSwitch() {
    //sets polarities

    
    m_forwardLimit = turretMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    m_reverseLimit = turretMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
  }

  public void enableLimitSwitch() {
    //enables limit switch

    m_forwardLimit.enableLimitSwitch(true);
    m_reverseLimit.enableLimitSwitch(true);

    SmartDashboard.putBoolean("Forward Limit Enabled", m_forwardLimit.isLimitSwitchEnabled());
    SmartDashboard.putBoolean("Reverse Limit Enabled", m_reverseLimit.isLimitSwitchEnabled());
  }

  @Override
  public void useOutput(final double output, final double setpoint) {
    m_turretPIDController.setReference(rotations, ControlType.kPosition);
  }

  public void setPosition() {
  SmartDashboard.putNumber("SetPoint", rotations);
  SmartDashboard.putNumber("ProcessVariable", turretEncoder.getPosition());
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }

  public void stop() {
    turretMotor.set(0);
  }
}