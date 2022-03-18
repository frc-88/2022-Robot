// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Turret;

public class TurretTrackWithGlobalPose extends CommandBase {
  private Turret m_turret;
  private final Navigation m_nav;
  private final String m_waypointName;
  
  /** Creates a new TurretTrackWithGlobalPose. */
  public TurretTrackWithGlobalPose(Turret turret, Navigation nav, String waypointName) {
    m_turret = turret;
    m_nav = nav;
    m_waypointName = waypointName;

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
      m_turret.goToDefaultFacing();
      return;
    }
    Pose2d target_map = m_nav.getWaypoint(m_waypointName);
    SmartDashboard.putBoolean("Turret:Is target valid", m_nav.isPoseValid(target_map));
    if (!m_nav.isPoseValid(target_map)) {
      m_turret.goToDefaultFacing();
      return;
    }
    Pose2d robot_pose = m_nav.getRobotPose();
    Translation2d target_base_link = robot_pose.getTranslation().minus(target_map.getTranslation());
    double target_global_angle = Math.atan2(target_base_link.getY(), target_base_link.getX());
    double target_angle = target_global_angle - robot_pose.getRotation().getRadians();
    SmartDashboard.putNumber("Turret:Track Target", Math.toDegrees(target_angle));
    SmartDashboard.putNumber("Turret:Target X", target_base_link.getX());
    SmartDashboard.putNumber("Turret:Target Y", target_base_link.getY());
    m_turret.goToFacing(Math.toDegrees(target_angle));
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
