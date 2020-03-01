/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Conveyor extends SubsystemBase {
  private CANSparkMax motor = new CANSparkMax(9, MotorType.kBrushed);
  private DigitalInput sensor = new DigitalInput(9);
    
  public Conveyor() {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.burnFlash();

  }

  @Override
  public void periodic() {
    if (RobotContainer.m_driverController.getRawButton(5))
      motor.set(-0.55);
    else if (RobotContainer.m_driverController.getRawButton(6))
    motor.set(-1);
    else if (RobotContainer.m_driverController.getRawButton(2))
    motor.set(0.75);
    // else if(!sensor.get())
    //   motor.set(-0.5);
    else
      motor.set(0);
  }
}
