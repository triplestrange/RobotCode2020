/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

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

  public Climb() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));

        

        new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
    getController().setTolerance(ShooterConstants.kShooterToleranceRPS);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
