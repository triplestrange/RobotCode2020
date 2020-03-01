/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final CANSparkMax shooter1, shooter2;
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  double speed = 0;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, kDf;

  public Shooter() {
    shooter1 = new CANSparkMax(12, MotorType.kBrushless);
    shooter2 = new CANSparkMax(13, MotorType.kBrushless);
    shooter1.restoreFactoryDefaults();
    shooter1.setIdleMode(IdleMode.kCoast);
    shooter1.setSmartCurrentLimit(80);
    shooter1.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    shooter1.burnFlash();
    shooter2.restoreFactoryDefaults();
    shooter2.setIdleMode(IdleMode.kCoast);
    shooter2.setSmartCurrentLimit(80);
    shooter2.follow(shooter1, true);
    shooter2.burnFlash();


    m_encoder = shooter1.getEncoder();
    m_pidController = shooter1.getPIDController();
    kP = 30;
    kI = 0;
    kD = 0; 
    // kDf = 0.5;
    kIz = 0; 
    kFF = 1.0/5700.0; 
    kMaxOutput = 1; 
    kMinOutput = 0.8;
    maxRPM = 5700.0;
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    // m_pidController.setDFilter(kDf);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("D Filter", kDf);
    SmartDashboard.putNumber("Shooter %", speed);
  }

  public void runShooter() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double df = SmartDashboard.getNumber("D filter", 0);
    double set = SmartDashboard.getNumber("Shooter %", 4750);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((df != kDf)) { m_pidController.setDFilter(df); kDf = df; }

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.ControlType.kDutyCycle
     *  com.revrobotics.ControlType.kPosition
     *  com.revrobotics.ControlType.kVelocity
     *  com.revrobotics.ControlType.kVoltage
     */
    double setPoint = set;
    m_pidController.setReference(setPoint, ControlType.kVelocity);
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    
  }
  public void stopShooter() {
    shooter1.set(0);
  }
  public void periodic() {

    SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
  }
}
