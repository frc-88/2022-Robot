// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import frc.robot.util.sensors.Limelight;

public class TurretTrack extends CommandBase {
  private Turret m_turret;
  private Limelight m_limelight;
  private double m_target;
  private double m_offset;
  private boolean m_circumnavigating;

  /** Creates a new TurretTrack. */
  public TurretTrack(Turret turret, Limelight limelight) {
    m_turret = turret;
    m_limelight = limelight;

    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_target = 0.0;
    m_circumnavigating = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_turret.isTracking()) {
      m_limelight.ledOn();

      if (m_circumnavigating) {
        m_circumnavigating = Math.abs(m_turret.getPosition() - m_target) > Constants.TURRET_SPIN_THRESHOLD;
      } else {
        if (m_limelight.hasTarget()) {
          // update offset if we have a target, otherwise, follow the last offset.
          m_offset = m_turret.turretDegreesToPosition(m_limelight.getTargetHorizontalOffsetAngle());
        }
        m_target = m_turret.getPosition() + m_offset;

        if (!m_turret.isPositionSafe(m_target)) {
          m_circumnavigating = true;
          m_target = Math.signum(m_target) * m_turret.turretDegreesToPosition(-360.0);
        }
      }
    } else { // not tracking
      // turn off the limelight and go to center position
      m_limelight.ledOff();
      m_target = 0.0;
    }
    m_turret.goToPosition(m_target);
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
