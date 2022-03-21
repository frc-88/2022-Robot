// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.sensors.Limelight;

public class TurretTrackLimelight extends CommandBase {
  private Turret m_turret;
  private Limelight m_limelight;
  private double m_target;
  private int m_lostCount = 0;
  private DoublePreferenceConstant p_resetTime = new DoublePreferenceConstant("Turret Tracking Reset Time", 2.0);

  /** Creates a new TurretTrack. */
  public TurretTrackLimelight(Turret turret, Limelight limelight) {
    m_turret = turret;
    m_limelight = limelight;

    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_target = m_turret.getDefaultFacing();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_turret.isTracking()) {
      m_limelight.ledOn();

      if (m_limelight.onTarget()) {
        m_lostCount = 0;
        m_target = m_turret.getFacing();
        // keep on same m_target
      } else if (m_limelight.hasTarget()) {
        // if we have a target, track it
        m_lostCount = 0;
        // TODO handle laggy data from the limelight
        m_target = m_turret.getFacing() - m_limelight.calcTurretOffset();
      } else if (++m_lostCount > (p_resetTime.getValue() * 50)) {
        // if we don't have a target for too long, go to zero
        m_target = m_turret.getDefaultFacing();
      }
    } else { // not tracking
      // turn off limelight, go to zero
      m_limelight.ledOff();
      m_target = m_turret.getDefaultFacing();
    }

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
