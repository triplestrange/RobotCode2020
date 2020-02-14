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
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  private final CANSparkMax mClimbL = new CANSparkMax(Constants.Climb.L_MOTOR, MotorType.kBrushless);
  private final CANSparkMax mClimbR = new CANSparkMax(Constants.Climb.R_MOTOR, MotorType.kBrushless);
  

  /**
   * Creates a new ClimbSubsystem.
   */
  public ClimbSubsystem() {
    mClimbL.setIdleMode(IdleMode.kBrake);
    mClimbR.setIdleMode(IdleMode.kBrake);
  }

  public void liftUp(final double speed) {
    mClimbL.set(speed);
    mClimbR.set(speed);
  }
  public void lowerDown(final double speed) {
    mClimbL.set(-speed);
    mClimbR.set(-speed);
  }

  public void stop() {
    mClimbL.set(0);
    mClimbR.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
