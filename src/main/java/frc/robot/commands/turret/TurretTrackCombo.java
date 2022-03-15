// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Turret;
import frc.robot.util.sensors.Limelight;

public class TurretTrackCombo extends CommandBase {
  private Turret m_turret;
  private final Navigation m_nav;
  private Limelight m_limelight;
  
  /** Creates a new TurretTrackWithGlobalPose. */
  public TurretTrackCombo(Turret turret, Navigation nav, Limelight limelight) {
    m_turret = turret;
    m_nav = nav;
    m_limelight = limelight;

    addRequirements(turret);
    addRequirements(nav);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_turret.isTracking()) {
      m_turret.goToFacing(0.0);
      m_limelight.ledOff();
      return;
    }
    
    m_limelight.ledOn();

    Pair<Double, Double> turret_target = TurretTargetResolver.getTurretTarget(m_nav, Navigation.CENTER_WAYPOINT_NAME, m_limelight, m_turret);

    m_turret.goToFacing(turret_target.getSecond());
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
