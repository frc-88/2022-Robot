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
import frc.robot.util.sensors.Limelight;

public class TurretTrackCombo extends CommandBase {
  private Turret m_turret;
  private final Navigation m_nav;
  private Limelight m_limelight;
  private final String m_waypointName;
  
  /** Creates a new TurretTrackWithGlobalPose. */
  public TurretTrackCombo(Turret turret, Navigation nav, Limelight limelight, String waypointName) {
    m_turret = turret;
    m_nav = nav;
    m_waypointName = waypointName;
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

    double limelight_target = Double.NaN;
    if (m_limelight.onTarget()) {
      // keep on same target
      limelight_target = m_turret.getFacing();
    }
    else if (m_limelight.hasTarget()) {
      // if we have a target, track it
      limelight_target = m_turret.getFacing() - m_limelight.calcTurretOffset();
    }

    Pose2d target_map = m_nav.getWaypoint(m_waypointName);
    SmartDashboard.putBoolean("Turret:Is target valid", m_nav.isPoseValid(target_map));
    if (!m_nav.isPoseValid(target_map)) {
      m_turret.goToFacing(0.0);
      return;
    }
    Pose2d robot_pose = m_nav.getRobotPose();
    Translation2d robot_relative_to_target = robot_pose.getTranslation().minus(target_map.getTranslation());
    double target_global_angle = Math.atan2(robot_relative_to_target.getY(), robot_relative_to_target.getX());
    double waypoint_target_angle = target_global_angle - robot_pose.getRotation().getRadians();
    waypoint_target_angle = Math.toDegrees(waypoint_target_angle);
    waypoint_target_angle = waypoint_target_angle % 360.0;
    
    double turret_target = 0.0;
    if (Double.isNaN(limelight_target)) {
      turret_target = waypoint_target_angle;
    }
    else {
      double limelight_global_delta = Math.abs((limelight_target % 360) - waypoint_target_angle);
      if (limelight_global_delta > 45.0) {
        turret_target = waypoint_target_angle;
      }
      else {
        turret_target = limelight_target;
      }
    }

    SmartDashboard.putNumber("Turret:Waypoint target", waypoint_target_angle);
    SmartDashboard.putNumber("Turret:Limelight target", limelight_target);
    SmartDashboard.putNumber("Turret:Track Target", turret_target);

    m_turret.goToFacing(Math.toDegrees(turret_target));
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
