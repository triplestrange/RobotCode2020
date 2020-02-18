package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.*;

public class OI {
    public static Joystick joy2 = new Joystick(0);
    public Button intake = new Button();
    public Button outtake = new Button();
    public static int DEFAULT_TIMEOUT = 30;

    public OI() {
 //       intake.whenPressed(new IntakeCommand(intake));
    
    }
}