/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Conveyor extends SubsystemBase {
  private CANSparkMax motor = new CANSparkMax(10, MotorType.kBrushed);
  public Conveyor() {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.burnFlash();
  }

  @Override
  public void periodic() {
    if (Math.abs(RobotContainer.joy2.getRawAxis(1))>0.1) {
      rollWheels(-RobotContainer.joy2.getRawAxis(1));
    }
    else if (RobotContainer.joy2.getRawButton(1)){
      rollWheels(-0.65);
    } else {
      rollWheels(0);
    }

  }

  public void rollWheels(double speed) {
    motor.set(speed);
}
}
