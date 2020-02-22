/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// dashboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// motor imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// color imports
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
// constants
import frc.robot.Constants;

public class ControlPanelSubsystem extends SubsystemBase {
	// Initialize constants
	// sparkmax for control panel motor
	private final CANSparkMax controlPanelMotor = new CANSparkMax(Constants.ControlPanel.Motor.bus_id, MotorType.kBrushless);
	// color sensor on onboard i2c port
	private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
	// create a color matcher
	private final ColorMatch colorMatcher = new ColorMatch();
	// setup colors from the robot constants
	private final Color Blue = ColorMatch.makeColor(Constants.ControlPanel.Colors.Blue.r, Constants.ControlPanel.Colors.Blue.g, Constants.ControlPanel.Colors.Blue.b);
	private final Color Green = ColorMatch.makeColor(Constants.ControlPanel.Colors.Green.r, Constants.ControlPanel.Colors.Green.g, Constants.ControlPanel.Colors.Green.b);
	private final Color Red = ColorMatch.makeColor(Constants.ControlPanel.Colors.Red.r, Constants.ControlPanel.Colors.Red.g, Constants.ControlPanel.Colors.Red.b);
	private final Color Yellow = ColorMatch.makeColor(Constants.ControlPanel.Colors.Yellow.r, Constants.ControlPanel.Colors.Yellow.g, Constants.ControlPanel.Colors.Yellow.b);

	public ControlPanelSubsystem() {
		controlPanelMotor.setIdleMode(IdleMode.kCoast);
		// add colors to color matcher
		colorMatcher.addColorMatch(Blue);
		colorMatcher.addColorMatch(Green);
		colorMatcher.addColorMatch(Yellow);
		colorMatcher.addColorMatch(Red);
	}

	/**
	 * Start the motor at a specified speed
	 * @param speed speed to spin the motor at
	 */
	public void startWheel(double speed) {
		controlPanelMotor.set(speed);
	}
	/**
	 * Stop the motor
	 */
	public void stopWheel() {
		controlPanelMotor.set(0);
	}

	/**
	 * Get the current color of the panel
	 * @return "blue", "red", "green", "yellow", or ""
	 */
	public String getCurrentColor() {
		// empty color string
		String colorString;
		// detect the current color
		Color detectedColor = colorSensor.getColor();
		// attempt to match our preset colors
		ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
		System.out.println(match.color);

		// if we matched a color set our string
		if (match.color == Blue) {
			colorString = "blue";
		} else if (match.color == Red) {
			colorString = "red";
		} else if (match.color == Green) {
			colorString = "green";
		} else if (match.color == Yellow) {
			colorString = "yellow";
		} else {
			colorString = "";
		}
	
		return colorString;
	}

	/**
	 * Write color data to the dashboard
	 */
	public void displayCurrentColor() {
		// detect the current color
		Color detectedColor = colorSensor.getColor();
		// attempt to match our preset colors
		ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
	
		// print raw RGB values to dashboard
		SmartDashboard.putNumber("Red", detectedColor.red);
		SmartDashboard.putNumber("Green", detectedColor.green);
		SmartDashboard.putNumber("Blue", detectedColor.blue);

		// print the detected color and the confidence value to the dashboard
		SmartDashboard.putString("Detected Color", getCurrentColor());
		SmartDashboard.putNumber("Confidence", match.confidence);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		// displayCurrentColor();
	}
}
