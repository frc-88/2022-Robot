// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Navigation;
import frc.robot.util.roswaypoints.WaypointMap;

public class SetGlobalPoseToWaypoint extends CommandBase {
  private final Navigation m_nav;
  private final String m_waypointName;
  private boolean m_validPoseReceived = false;
  private int m_timeCounter = -1;
  /** Creates a new SetGlobalPoseToWaypoint. */
  public SetGlobalPoseToWaypoint(Navigation nav, String waypointName) {
    m_nav = nav;
    m_waypointName = waypointName;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_nav);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_validPoseReceived = false;
    m_timeCounter = -1;
  }

  @Override
  public void execute() {
    m_timeCounter++;
    if (m_timeCounter % 20 != 0) {
      return;
    }
    Pose2d pose = m_nav.getWaypoint(m_waypointName);
    if (m_nav.isPoseValid(pose)) {
      m_nav.setPoseEstimate(pose);
      m_validPoseReceived = true;
      SmartDashboard.putBoolean("ROS start pose valid", true);
    }
    else {
      switch (WaypointMap.parseWaypointName(m_waypointName)) {
        case "blue_start_5": m_nav.setPoseEstimate(new Pose2d(-0.5706, -2.3307, new Rotation2d(-1.5113))); break;
        case "blue_start_2": m_nav.setPoseEstimate(new Pose2d(-2.2705, 1.1093, new Rotation2d(2.4611))); break;
        case "red_start_5": m_nav.setPoseEstimate(new Pose2d(0.5706, 2.3307, new Rotation2d(1.6303))); break;
        case "red_start_2": m_nav.setPoseEstimate(new Pose2d(2.2705, -1.1093, new Rotation2d(-0.68053))); break;
        default: break;
      }
      SmartDashboard.putBoolean("ROS start pose valid", false);
      System.out.println("Warning: " + m_waypointName + " is not a valid waypoint name");
    }
  }

  public boolean isFinished() {
    return m_validPoseReceived;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}