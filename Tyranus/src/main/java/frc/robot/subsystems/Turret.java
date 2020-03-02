/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

public class Turret extends SubsystemBase {
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
   
  public Turret() {
    super();
      m_forwardLimit = turretMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
      turretMotor = new CANSparkMax(deviceID, MotorType.kBrushless);
      turretMotor.restoreFactoryDefaults();
      m_turretPIDController = turretMotor.getPIDController();
      turretEncoder = turretMotor.getEncoder();
      disableLimitSwitch();
      SmartDashboard.putBoolean("Limit Enabled", m_forwardLimit.isLimitSwitchEnabled());
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
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_turretPIDController.setP(p); kP = p; }
    if((i != kI)) { m_turretPIDController.setI(i); kI = i; }
    if((d != kD)) { m_turretPIDController.setD(d); kD = d; }
    if((iz != kIz)) { m_turretPIDController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_turretPIDController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_turretPIDController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 

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
  }
  
  public void resetLimitSwitch() {
    //sets polarities
    m_forwardLimit = turretMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
  }

  public boolean enableLimitSwitch() {
    m_forwardLimit.enableLimitSwitch(true);
    return true;
  }

  public boolean disableLimitSwitch() {
    m_forwardLimit.enableLimitSwitch(false);
    return false;
  }

  public boolean getLimitSwitchStatus() {
    if (enableLimitSwitch()) {
      return true;
    }
    else 
      return false;
  }
  private Joystick m_stick;
  public void moveTurret() {
    turretMotor.set(m_stick.getY());
    turretMotor.set(speed);


  }
  
  public void setPosition() {
  SmartDashboard.putNumber("SetPoint", rotations);
  SmartDashboard.putNumber("ProcessVariable", turretEncoder.getPosition());
  }

  public void stop() {
    turretMotor.set(0);
  }
}