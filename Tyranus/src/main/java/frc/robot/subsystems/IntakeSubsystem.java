package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax mIntake = new CANSparkMax(Constants.Intake.MOTOR, MotorType.kBrushless);
    private Solenoid intakeSolenoid = new Solenoid(0);

    public IntakeSubsystem() {
        super();
       
        mIntake.setIdleMode(IdleMode.kCoast);
    }

    public void extend() {
        intakeSolenoid.set(true);
    }

    public void retract() {
        intakeSolenoid.set(false);
    }

    public void rollWheels(double speed) {
        mIntake.set(speed);
    }

    public void stop() {
        rollWheels(0);
    }

}