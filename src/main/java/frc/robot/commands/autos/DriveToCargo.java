// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Navigation.RosAutoState;
import frc.robot.util.roswaypoints.Waypoint;
import frc.robot.util.roswaypoints.WaypointsPlan;
import frc.robot.util.ThisRobotTable;
import frc.robot.util.coprocessortable.VelocityCommand;

public class DriveToCargo extends CommandBase {
  private final Navigation m_nav;
  private final Drive m_drive;
  private final Sensors m_sensors;
  private final ThisRobotTable m_robot_table;
  private WaypointsPlan m_plan;
  private final String m_waypointName = "cargo_<team>";
  private long m_is_finished_timeout = 0;

  /** Creates a new DriveToWaypoint. */
  public DriveToCargo(Navigation nav, ThisRobotTable robot_table, Drive drive, Sensors sensors, long is_finished_timeout) {
    m_nav = nav;
    m_drive = drive;
    m_sensors = sensors;
    m_robot_table = robot_table;
    m_is_finished_timeout = is_finished_timeout;
    m_plan = new WaypointsPlan(m_nav.getCoprocessorTable());
    m_plan.addWaypoint(new Waypoint(m_waypointName));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(nav);
    addRequirements(drive);
  }
  public DriveToCargo(Navigation nav, ThisRobotTable robot_table, Drive drive, Sensors sensors) {
    this(nav, robot_table, drive, sensors, 15_000_000);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_nav.setWaypointsPlan(m_plan, m_is_finished_timeout);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    VelocityCommand command = m_nav.getAutoCommand();
    if (Objects.isNull(command)) {
      return;
    }
    m_drive.drive(command);
    m_sensors.setCameraTilterAngle(m_robot_table.getCameraTiltCommand());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_nav.cancelAutoGoal();  // Confirm whether cancelGoal should be called in all cases
    m_sensors.setCameraTilterAngle(Constants.CAMERA_TILT_DOWN_ANGLE);
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_nav.isRosAutoFinished();
  }
}
