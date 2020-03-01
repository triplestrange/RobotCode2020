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
  public double kP, kFF, kMaxOutput, kMinOutput, maxRPM, setPoint, speed;

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
    kFF = 1.0/5700.0; 
    kMaxOutput = 1; 
    kMinOutput = 0.8;
    maxRPM = 5700.0;
    speed = 0;
    m_pidController.setP(kP);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    SmartDashboard.putNumber("Shooter P", kP);
    SmartDashboard.putNumber("Shooter Velocity", speed);
  }

  public void runShooter() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("Shooter P", 0);
    setPoint = SmartDashboard.getNumber("Shooter Velocity", 4750);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    m_pidController.setReference(setPoint, ControlType.kVelocity);
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    
  }
  public void stopShooter() {
    shooter1.set(0);
  }
  public void periodic() {
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
  }
  public boolean atSpeed() {
    return Math.abs(setPoint - m_encoder.getVelocity())<30;
  }
}
