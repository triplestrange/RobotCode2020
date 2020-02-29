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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final CANSparkMax shooter1, shooter2;
  double speed = 0;

  public Shooter() {
    shooter1 = new CANSparkMax(12, MotorType.kBrushless);
    shooter2 = new CANSparkMax(13, MotorType.kBrushless);
    shooter1.restoreFactoryDefaults();
    shooter1.setIdleMode(IdleMode.kCoast);
    shooter1.setSmartCurrentLimit(80);
    shooter1.burnFlash();
    shooter2.restoreFactoryDefaults();
    shooter2.setIdleMode(IdleMode.kCoast);
    shooter2.setSmartCurrentLimit(80);
    shooter2.burnFlash();

    SmartDashboard.putNumber("Shooter %", speed);
  }

  @Override
  public void periodic() {
    
    double set = SmartDashboard.getNumber("Shooter %", 0);
    if ((set != speed)) {
      speed = set;
    }
      shooter1.set(speed);
      shooter2.set(-speed);
    
  }
}
