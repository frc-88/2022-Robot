// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.util.sensors.Limelight;

public class TurretTrack extends CommandBase {
  private Turret m_turret;
  private Limelight m_limelight;
  private double m_targetOffset;

  /** Creates a new TurretTrack. */
  public TurretTrack(Turret turret, Limelight limelight) {
    m_turret = turret;
    m_limelight = limelight;

    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetOffset = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_turret.isTracking()) {
      m_limelight.ledOn();
      
      // update m_targetOffset if there is one, follow the last offset if not
      if(m_limelight.hasTarget()) {
        m_targetOffset = m_limelight.getTargetHorizontalOffsetAngle();
      }

      // TODO if new target position in "dangerzone" do 360
      
      m_turret.goToPositionRelative(m_targetOffset);
    } else { // not tracking
      // turn off the limelight and go to center position
      m_limelight.ledOff();
      m_turret.goToPosition(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
