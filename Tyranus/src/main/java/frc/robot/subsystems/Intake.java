package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
    private CANSparkMax intakeMotor = new CANSparkMax(11, MotorType.kBrushless);
    // private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(0,1);

    public Intake() {
        super();
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.enableVoltageCompensation(11);
        intakeMotor.setSmartCurrentLimit(20);
        intakeMotor.burnFlash();
    }

    public void extend() {
        // intakeSolenoid.set(Value.kForward);
    }

    public void retract() {
        // intakeSolenoid.set(Value.kReverse);
    }

    public void periodic() {
        intakeMotor.set(RobotContainer.m_driverController.getRawAxis(2)/2.0);
    }

}