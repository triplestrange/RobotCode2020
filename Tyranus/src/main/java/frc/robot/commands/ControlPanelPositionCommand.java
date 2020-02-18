/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

import frc.robot.Constants;

/**
 * Control Panel command
 */
public class ControlPanelPositionCommand extends CommandBase {
	private final ControlPanelSubsystem _subsystem;

	// string to store the sensors target color
	private String targetColor = "";

	/**
	 * @param subsystem The subsystem used by this command.
	 */
	public ControlPanelPositionCommand(ControlPanelSubsystem subsystem) {
		_subsystem = subsystem;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// get game data from the driverstation
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		// while we haven't determined the target color
		while (targetColor == "") {
			// if we have available game data
			if(gameData.length() > 0) {
				// set the sensor's target color
				//   convert the game data character to a string,
				//   then read the target color from the color hash map at that string
				targetColor = Constants.ControlPanel.ColorMap.get(String.valueOf(gameData.charAt(0)));
				// start spinning at the defined speed
				_subsystem.startWheel(Constants.ControlPanel.Motor.speed);
			}
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// stop spinning when the command ends
		_subsystem.stopWheel();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// if we've spun past the predetermined number of sections, end
		if (_subsystem.getCurrentColor() == targetColor) {
			return true;
		} else {
			return false;
		}
	}
}
