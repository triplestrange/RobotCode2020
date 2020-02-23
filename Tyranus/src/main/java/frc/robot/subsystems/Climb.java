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
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

import com.revrobotics.*;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Climb extends PIDSubsystem {
  /**
   * Creates a new Climb.
   */


        private final CANSparkMax mClimbL = new CANSparkMax(Constants.Climb.motorL, MotorType.kBrushless);
        private final CANSparkMax mClimbR = new CANSparkMax(Constants.Climb.motorR, MotorType.kBrushless);
        private final Solenoid climbSolenoid = new Solenoid(0);
        private final boolean climb_elevator = false;
        private CANEncoder m_climbEncoder;
        private CANPIDController m_pidController;
        public static double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

        
        m_climbEncoder = (Climb.kEncoderPorts[0], Climb.kEncoderPorts[1], Climb.kEncoderReversed);

        private final SimpleMotorFeedforward m_shooterFeedforward = 
              new SimpleMotorFeedforward(Climb.kSVolts, Climb.kVVoltsSecondsPerRotation);

  public Climb() {
    super(
        // The PIDController used by the subsystem
        new PIDController(Climb.kP, Climb.kI, Climb.kD));
        getController().setTolerance(Climb.ShooterToleranceRPS);

    mClimbL.setIdleMode(IdleMode.kCoast);
    mClimbR.setIdleMode(IdleMode.kCoast);
    climbSolenoid.set(false);
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

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_climbEncoder.getRate();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

}
