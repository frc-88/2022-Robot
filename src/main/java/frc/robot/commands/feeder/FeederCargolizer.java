// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.util.CargoSource;
import frc.robot.util.CargoTarget;

public class FeederCargolizer extends CommandBase {
  private Feeder m_feeder;
  private CargoSource m_source;
  private CargoTarget m_target;
  private boolean m_sawOne, m_cargoComing;
  private int m_waitingCount;

  /** Creates a new FeederCargolizer. */
  public FeederCargolizer(Feeder feeder, CargoSource source, CargoTarget target) {
    m_feeder = feeder;
    m_source = source;
    m_target = target;

    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_sawOne = false;
    m_cargoComing = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_feeder.hasCargo()) {
      m_sawOne = true;
      m_cargoComing = false;
      if (m_target.wantsCargo()) {
        m_feeder.run();
      } else {
        m_feeder.stop();
      }
    } else if (m_source.hasCargo()) {
      m_cargoComing = true;
      m_waitingCount = 0;
      m_feeder.run();
    } else if (m_cargoComing && m_waitingCount++<25) {
      m_feeder.run();
    } else {
      m_cargoComing = false;
      if (m_sawOne) {
        m_feeder.idle();
      } else {
        m_feeder.stop();
      }
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
