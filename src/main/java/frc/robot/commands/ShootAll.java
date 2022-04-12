// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.util.CargoSource;

public class ShootAll extends CommandBase {
  /** Creates a new ShootAll. */
  private Shooter m_shooter;
  private Integer m_count;
  private boolean m_ballSeen;

  public ShootAll(Shooter shooter) {
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_count = 0;
    m_ballSeen = false;
    m_shooter.activatePermissive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_shooter.sourcesHaveCargo() && m_ballSeen) {
      m_count++;
    } else {
      if (m_shooter.sourcesHaveCargo()) {
        m_ballSeen = true;
        m_count = 0;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.deactivate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_count > 25;
  }
}
