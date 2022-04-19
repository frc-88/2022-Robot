// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Targeting;
import frc.robot.subsystems.Turret;

public class TurretTrackCombo extends CommandBase {
  private Targeting m_targeting;
  private Turret m_turret;
  
  /** Creates a new TurretTrackWithGlobalPose. */
  public TurretTrackCombo(Turret turret, Targeting targeting) {
    m_targeting = targeting;
    m_turret = turret;

    addRequirements(m_turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_targeting.isTracking()) {
      m_targeting.disableTurret();
      m_turret.setHasTarget(false);
      return;
    }
    
    m_targeting.enableTurret();
    m_turret.goToFacing(m_targeting.getTurretAngle(), true);
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
