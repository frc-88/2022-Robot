// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Navigation;

public class SetGlobalPoseToWaypoint extends CommandBase {
  private final Navigation m_nav;
  private final String m_waypointName;
  /** Creates a new SetGlobalPoseToWaypoint. */
  public SetGlobalPoseToWaypoint(Navigation nav, String waypointName) {
    m_nav = nav;
    m_waypointName = waypointName;
    // addRequirements(m_nav);
    // // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d pose = m_nav.getWaypoint(m_waypointName);
    if (m_nav.isPoseValid(pose)) {
      m_nav.setPoseEstimate(pose);
    }
    else {
      System.out.println("Warning: " + m_waypointName + " is not a valid waypoint name");
    }
  }

  public boolean isFinished() {
    return true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
