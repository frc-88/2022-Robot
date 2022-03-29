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
  private boolean m_cargoComing;
  private int m_waitingCount;
  private int m_cargoFoundCount;

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
    m_cargoComing = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_feeder.hasCargo()) {
      m_cargoComing = false;
      if (m_target.wantsCargo()) {
        m_feeder.forceForwards();
      } else if (m_cargoFoundCount++ < 50) {
        m_feeder.runUntilBallFound();
      } else {
        m_feeder.stop();
      }
    } else if (m_source.hasCargo()) {
      m_cargoComing = true;
      m_waitingCount = 0;
      m_cargoFoundCount = -1;
      m_feeder.runUntilBallFound();
    } else if (m_cargoComing && m_waitingCount++<100) {
      m_cargoFoundCount = -1;
      m_feeder.runUntilBallFound();
    } else {
      m_cargoFoundCount = -1;
      m_cargoComing = false;
      m_feeder.stop();
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
