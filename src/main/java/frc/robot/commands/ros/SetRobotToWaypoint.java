// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ros;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.roswaypoints.WaypointMap;
import frc.robot.util.tunnel.ROSInterface;

public class SetRobotToWaypoint extends InstantCommand {
  private ROSInterface m_ros_interface;
  private WaypointMap m_waypoint_map;
  private String m_waypoint_name;
  
  /** Creates a new SetRobotToWaypoint. */
  public SetRobotToWaypoint(String waypoint_name, ROSInterface ros_interface, WaypointMap waypoint_map) {
    m_ros_interface = ros_interface;
    m_waypoint_map = waypoint_map;
    m_waypoint_name = waypoint_name;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_waypoint_map.doesWaypointExist(m_waypoint_name)) {
      Pose2d pose = m_waypoint_map.getWaypoint(m_waypoint_name);
      m_ros_interface.setPoseEstimate(pose);
    }
    else {
      System.out.println(m_waypoint_name + " does not exist. Skipping set robot pose");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
