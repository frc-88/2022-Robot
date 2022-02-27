// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Coprocessor;
import frc.robot.subsystems.Coprocessor.RosAutoState;
import frc.robot.util.roswaypoints.Waypoint;
import frc.robot.util.roswaypoints.WaypointsPlan;

public class DriveToShootingZone extends CommandBase {
  private Coprocessor m_coprocessor;
  /** Creates a new DriveToShootingZone. */
  public DriveToShootingZone(Coprocessor coprocessor) {
    m_coprocessor = coprocessor;
    addRequirements(m_coprocessor);
    addRequirements(m_coprocessor.getDriveSubsystem());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d goal_pose = m_coprocessor.calculateNearestShootingZonePose(Constants.SHOOTING_ZONE_INNER_RADIUS_METERS, Constants.SHOOTING_ZONE_OUTER_RADIUS_METERS);
    WaypointsPlan plan = m_coprocessor.makeWaypointPlan();
    plan.addWaypoint(new Waypoint(goal_pose));
    m_coprocessor.setWaypointsPlan(plan, 20_000_000);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coprocessor.cancelAutoGoal();  // Confirm whether cancelGoal should be called in all cases
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_coprocessor.getRosAutoState() == RosAutoState.FINISHED;
  }
}
