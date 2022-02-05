// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.util.TJController;
import frc.robot.util.drive.DriveUtils;

public class ManualModeClimber extends CommandBase {
  private Climber m_climber;
  private TJController m_tjController;
  /** Creates a new ManualModeClimber. */
  public ManualModeClimber(Climber climber, TJController tjController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    addRequirements(climber);
    m_tjController = tjController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double LeftStickX = DriveUtils.deadbandExponential(m_tjController.getLeftStickX(), 3, 0.25);
    double LeftStickY = DriveUtils.deadbandExponential(m_tjController.getLeftStickY(), 3, 0.25);
    double RightStickX = DriveUtils.deadbandExponential(m_tjController.getRightStickX(), 3, 0.25);
    double RightStickY = DriveUtils.deadbandExponential(m_tjController.getRightStickY(), 3, 0.25);
    m_climber.setInnerPercentOutput(LeftStickX, LeftStickY);
    m_climber.setOuterPercentOutput(RightStickX, RightStickY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setInnerPercentOutput(0, 0);
    m_climber.setOuterPercentOutput(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
