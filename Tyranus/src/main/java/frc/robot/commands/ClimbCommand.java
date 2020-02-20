/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends CommandBase {
  private final ClimbSubsystem m_climb;
  private int m_dir;
  public double y;
  /**
   * Creates a new ClimbCommand.
   */
  public ClimbCommand(ClimbSubsystem climb) {
    m_climb = climb;
     m_dir = -1;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
  double y = Constants.joy2.getRawAxis(Constants.Controller.LY);
    if (Math.abs(y) > 0.15) {
      m_climb.liftUp(y);
    } else if (Math.abs(y) < 0.15) {
      m_climb.lowerDown(y);
    }
}

  public int getDir() {
    return m_dir;
  }

  public void setDir(int dir) {
    m_dir = dir;
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
