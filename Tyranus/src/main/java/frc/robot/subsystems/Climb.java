/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.*;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Climb extends SubsystemBase {
  /**
   * Creates a new Climb.
   */       
        private final boolean climb_elevator = false;
        private CANEncoder m_climbEncoder;
        private CANPIDController m_climbPIDController;
        public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, rotations;

        //initialize motors
        private final CANSparkMax mClimb = new CANSparkMax(Constants.ClimbConstants.motorL, MotorType.kBrushless);
        
        private final Solenoid climbSolenoid = new Solenoid(0);
        
        
  public Climb() {
    super();

    // PID coefficients
    kP = 1;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;
        

    mClimb.restoreFactoryDefaults();
    climbSolenoid.set(false);

    m_climbPIDController = mClimb.getPIDController();

    m_climbEncoder = mClimb.getEncoder();


    }

  public void displayPID() {
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
  }


  public void setPID() {
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    rotations = SmartDashboard.getNumber("Set Rotations", 0);

    if((p != kP)) { m_climbPIDController.setP(p); kP = p; }
    if((i != kI)) { m_climbPIDController.setI(i); kI = i; }
    if((d != kD)) { m_climbPIDController.setD(d); kD = d; }
    if((iz != kIz)) { m_climbPIDController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_climbPIDController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_climbPIDController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

  // set PID coefficients
    m_climbPIDController.setP(kP);
    m_climbPIDController.setI(kI);
    m_climbPIDController.setD(kD);
    m_climbPIDController.setIZone(kIz);
    m_climbPIDController.setFF(kFF);
    m_climbPIDController.setOutputRange(kMinOutput, kMaxOutput);
  }

  public void setPosition() {
    m_climbPIDController.setReference(rotations, ControlType.kPosition);
    
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", m_climbEncoder.getPosition());
  }


    
  public void liftToPosition(int rotations) {
    m_climbPIDController.setReference(rotations, ControlType.kPosition);
    climbSolenoid.set(true);
  }

  public void lowerToPosition(int rotations) {
    m_climbPIDController.setReference(rotations, ControlType.kPosition);
    climbSolenoid.set(false);
  }
  

  public void stop() {
    mClimb.set(0);
    climbSolenoid.set(false);
  }

}
