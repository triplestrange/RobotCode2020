/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeNEW extends SubsystemBase {
  /**
   * Creates a new IntakeNEW.
   */

  private CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.motor, MotorType.kBrushless);
  private DoubleSolenoid solenoid = new DoubleSolenoid(0, 1);

  public IntakeNEW() {
    
  }

  public void extend() {
    solenoid.set(Value.kForward);
  }

  public void retract() {
    solenoid.set(Value.kReverse);
  }

  public void runWheels() {
    intakeMotor.set(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
