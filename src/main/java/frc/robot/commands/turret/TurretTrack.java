// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

      if (m_limelight.onTarget()) {
        // stay on target
      } else if (m_limelight.hasTarget()) {
        // calculate offset from target
        double distance = m_limelight.calcDistanceToTarget();
        double angle = m_limelight.getTargetHorizontalOffsetAngle();

        m_offset = Math.atan((distance * Math.sin(angle)) /
            (distance * Math.cos(angle) - Constants.LIMELIGHT_TURRET_RADIUS));

        m_target = m_turret.getFacing() - m_offset;
      }
    } else { // not tracking
      // turn off limelight, hold position
      m_limelight.ledOff();
    }

    SmartDashboard.putNumber("Turret:Track Offset", m_offset);
    SmartDashboard.putNumber("Turret:Track Target", m_target);
    
    m_turret.goToFacing(m_target);
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
