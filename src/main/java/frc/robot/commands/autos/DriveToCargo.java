// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Navigation.RosAutoState;
import frc.robot.util.roswaypoints.Waypoint;
import frc.robot.util.roswaypoints.WaypointsPlan;
import frc.robot.util.ThisRobotTable;
import frc.robot.util.coprocessortable.VelocityCommand;

public class DriveToCargo extends CommandBase {
  private final Navigation m_nav;
  private final SwerveDrive m_drive;
  private final Shooter m_shooter;
  private final Sensors m_sensors;
  private final ThisRobotTable m_robot_table;
  private WaypointsPlan m_plan;
  private final String m_waypointName = "cargo_<team>";
  private long m_is_finished_timeout = 0;
  private int m_numCargo = 1;

  /** Creates a new DriveToWaypoint. */
  public DriveToCargo(Navigation nav, ThisRobotTable robot_table, SwerveDrive drive, Shooter shooter, Sensors sensors, long is_finished_timeout, int numCargo) {
    m_nav = nav;
    m_drive = drive;
    m_sensors = sensors;
    m_shooter = shooter;
    m_robot_table = robot_table;
    m_numCargo = numCargo;
    m_is_finished_timeout = is_finished_timeout;
    m_plan = new WaypointsPlan(m_nav.getCoprocessorTable());
    for (int count = 0; count < m_numCargo; count++) {
      m_plan.addWaypoint(new Waypoint(m_waypointName));
    }

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(nav);
    addRequirements(drive);
  }
  public DriveToCargo(Navigation nav, ThisRobotTable robot_table, SwerveDrive drive, Shooter shooter, Sensors sensors, int numCargo) {
    this(nav, robot_table, drive, shooter, sensors, 15_000_000, numCargo);
  }

  public DriveToCargo(Navigation nav, ThisRobotTable robot_table, SwerveDrive drive, Shooter shooter, Sensors sensors) {
    this(nav, robot_table, drive, shooter, sensors, 15_000_000, 1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.activateRestrictive();
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
    m_sensors.setCameraTilterAngle(Constants.CAMERA_TILT_DOWN_ANGLE);
    // m_sensors.setCameraTilterAngle(m_robot_table.getCameraTiltCommand());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_nav.cancelAutoGoal();  // Confirm whether cancelGoal should be called in all cases
    m_sensors.setCameraTilterAngle(Constants.CAMERA_TILT_DOWN_ANGLE);
    m_drive.stop();
    m_shooter.deactivate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_nav.isRosAutoFinished();
  }
}
