/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

  private final IntakeSubsystem m_intake;
  private int m_dir;
  public double y;
    /**
   * Creates a new Intake.
   */
  public IntakeCommand(IntakeSubsystem intake) {
    m_intake = intake;
    m_dir = -1;
    addRequirements(intake);
  }

  public IntakeCommand(IntakeSubsystem intake, int dir) {
    m_intake = intake;
    dir = -1;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_dir == 1){
      y = OI.joy2.getRawAxis(Constants.Controller.LT);
      m_intake.rollWheels(y);
  }
    else if (m_dir == -1) {
      y = OI.joy2.getRawAxis(Constants.Controller.RT);
      m_intake.rollWheels(y);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
