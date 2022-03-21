// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Navigation.RosAutoState;
import frc.robot.util.roswaypoints.Waypoint;
import frc.robot.util.roswaypoints.WaypointsPlan;
import frc.robot.util.coprocessortable.VelocityCommand;

public class DriveToDonutZone extends CommandBase {
  private final Navigation m_nav;
  private final Drive m_drive;
  private long m_is_finished_timeout = 0;

  /** Creates a new DriveToDonutZone. */
  public DriveToDonutZone(Navigation nav, Drive drive, long is_finished_timeout) {
    m_nav = nav;
    m_drive = drive;
    m_is_finished_timeout = is_finished_timeout;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(nav);
  }
  public DriveToDonutZone(Navigation nav, Drive drive) {
    this(nav, drive, 15_000_000);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d goal_pose = m_nav.calculateNearestShootingZonePose(Constants.SHOOTING_ZONE_INNER_RADIUS_METERS, Constants.SHOOTING_ZONE_OUTER_RADIUS_METERS);
    WaypointsPlan plan = m_nav.makeEmptyWaypointPlan();
    plan.addWaypoint(new Waypoint(goal_pose));
    m_nav.setWaypointsPlan(plan, m_is_finished_timeout);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    VelocityCommand command = m_nav.getAutoCommand();
    if (Objects.isNull(command)) {
      return;
    }
    m_drive.drive(command);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_nav.cancelAutoGoal();  // Confirm whether cancelGoal should be called in all cases
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_nav.getRosAutoState() == RosAutoState.FINISHED;
  }
}
