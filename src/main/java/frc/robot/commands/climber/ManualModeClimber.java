// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.util.controllers.XboxController;
import frc.robot.util.drive.DriveUtils;

public class ManualModeClimber extends CommandBase {
  private Climber m_climber;
  private XboxController m_tjController;
  /** Creates a new ManualModeClimber. */
  public ManualModeClimber(Climber climber, XboxController tjController) {
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
    double leftStickX = DriveUtils.deadbandExponential(m_tjController.getLeftStickX(), 3, 0.25);
    double leftStickY = DriveUtils.deadbandExponential(m_tjController.getLeftStickY(), 3, 0.25);
    double rightStickX = DriveUtils.deadbandExponential(m_tjController.getRightStickX(), 3, 0.25);
    double rightStickY = DriveUtils.deadbandExponential(m_tjController.getRightStickY(), 3, 0.25);
    m_climber.setInnerPercentOutput(leftStickX, leftStickY);
    m_climber.setOuterPercentOutput(rightStickX, rightStickY);
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
