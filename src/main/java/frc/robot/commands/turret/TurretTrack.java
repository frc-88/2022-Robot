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

  /** Creates a new TurretTrack. */
  public TurretTrack(Turret turret, Limelight limelight) {
    m_turret = turret;
    m_limelight = limelight;

    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Turn on limelight
    if (m_turret.isTracking()) {
      m_limelight.ledOn();
    // if limelight has target, move towards it
    // else if target never seen go to zero
    // else move in direction target was last seen
    // 
    // if new target position in "dangerzone" do 360
  } else {
      m_limelight.ledOff();
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
