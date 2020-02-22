/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

import frc.robot.Constants;

/**
 * Control Panel command
 */
public class ControlPanelRotationCommand extends CommandBase {
	private final ControlPanelSubsystem m_subsystem;

	// total spin count
	private final int spins = 4;
	// calculate remaining sections from the spin count
	private int remainingSections = spins * 8;
	// string to store and compare the current color
	private String currentColor = "";

	/**
	 * @param subsystem The subsystem used by this command.
	 */
	public ControlPanelRotationCommand(ControlPanelSubsystem subsystem) {
		m_subsystem = subsystem;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// start spinning at the defined speed
		m_subsystem.startWheel(Constants.ControlPanel.Motor.speed);
		// read in the current color
		currentColor = m_subsystem.getCurrentColor();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// if the current color has changed ( doesn't match our stored reading )
		if (currentColor != m_subsystem.getCurrentColor()) {
			// update our stored color
			currentColor = m_subsystem.getCurrentColor();
			// increment the sections counter
			remainingSections = remainingSections - 1;
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// stop spinning when the command ends
		m_subsystem.stopWheel();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// if we've spun past the predetermined number of sections, end
		if (remainingSections < 1) {
			return true;
		} else {
			return false;
		}
	}
}
