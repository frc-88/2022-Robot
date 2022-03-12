// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Turret;

public class TurretTrackWithGlobalPose extends CommandBase {
  private Turret m_turret;
  private final Navigation m_nav;
  
  /** Creates a new TurretTrackWithGlobalPose. */
  public TurretTrackWithGlobalPose(Turret turret, Navigation nav) {
    m_turret = turret;
    m_nav = nav;

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
      return;
    }
    Pose2d target_map = m_nav.getCenterWaypoint();
    if (!m_nav.isPoseValid(target_map)) {
      return;
    }
    Pose2d robot_pose = m_nav.getRobotPose();
    Pose2d target_base_link = target_map.relativeTo(robot_pose);
    double target_angle = Math.toDegrees(Math.atan2(target_base_link.getY(), target_base_link.getX()));
    SmartDashboard.putNumber("Turret:Track Target", target_angle);
    m_turret.goToFacing(target_angle);
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
