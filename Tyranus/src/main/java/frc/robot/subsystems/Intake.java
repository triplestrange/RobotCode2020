package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
    private CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.MOTOR, MotorType.kBrushless);
    private Solenoid intakeSolenoid = new Solenoid(0);

    public Intake() {
        super();
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeMotor.enableVoltageCompensation(11);
        intakeMotor.setSmartCurrentLimit(20);
        intakeMotor.burnFlash();
    }

    public void extend() {
        intakeSolenoid.set(true);
    }

    public void retract() {
        intakeSolenoid.set(false);
    }

    public void rollWheels(double speed) {
        intakeMotor.set(speed);
    }

    public void stop() {
        rollWheels(0);
    }

}