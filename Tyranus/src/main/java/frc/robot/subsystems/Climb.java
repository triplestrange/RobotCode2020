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
        public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

        //initialize motors
        private final CANSparkMax mClimbL = new CANSparkMax(Constants.ClimbConstants.motorL, MotorType.kBrushless);
        private final CANSparkMax mClimbR = new CANSparkMax(Constants.ClimbConstants.motorR, MotorType.kBrushless);

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
        

    mClimbL.restoreFactoryDefaults();
    mClimbR.restoreFactoryDefaults();
    climbSolenoid.set(false);

    m_climbPIDController = mClimbL.getPIDController();

    m_climbEncoder = mClimbL.getEncoder();


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
  // set PID coefficients
    m_climbPIDController.setP(kP);
    m_climbPIDController.setI(kI);
    m_climbPIDController.setD(kD);
    m_climbPIDController.setIZone(kIz);
    m_climbPIDController.setFF(kFF);
    m_climbPIDController.setOutputRange(kMinOutput, kMaxOutput);
  }

  public void liftUp(final double speed) {
    mClimbL.set(speed);
    mClimbR.set(speed);
    climbSolenoid.set(true);
  }
  public void lowerDown(final double speed) {
    mClimbL.set(-speed);
    mClimbR.set(-speed);
    climbSolenoid.set(false);
  }

  public void stop() {
    mClimbL.set(0);
    mClimbR.set(0);
    climbSolenoid.set(false);
  }

  

}
