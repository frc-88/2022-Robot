// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.ThisRobotTable;

public class SetGlobalPoseToLimelight extends CommandBase {
  private final ThisRobotTable m_ros_interface;
  /** Creates a new SetGlobalPoseToLimelight. */
  public SetGlobalPoseToLimelight(ThisRobotTable ros_interface) {
    m_ros_interface = ros_interface;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(m_ros_interface);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ros_interface.resetPoseToLimelight();
  }

  public boolean isFinished() {
    return true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
