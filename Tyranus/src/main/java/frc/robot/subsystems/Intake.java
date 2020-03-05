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

    private CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.motor, MotorType.kBrushless);
    private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(0, 1);
    private boolean extended = false;

    public Intake() {
        super();
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.enableVoltageCompensation(11);
        intakeMotor.setSmartCurrentLimit(20);
        intakeMotor.burnFlash();
    }

    public void extend() {
        intakeSolenoid.set(Value.kForward);
        extended = true;
    }

    public void retract() {
        intakeSolenoid.set(Value.kReverse);
        extended = false;
    }

    public void runWheels(double speedIn, double speedOut) {
        // if (extended) {
            if (speedIn > 0.1)
                intakeMotor.set(speedIn / 3);
            else if (speedOut > 0.1)
                intakeMotor.set(-speedOut / 3);
         else
            intakeMotor.set(0);
    }

}