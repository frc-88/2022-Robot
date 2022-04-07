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
import frc.robot.util.roswaypoints.Waypoint;
import frc.robot.util.roswaypoints.WaypointsPlan;
import frc.robot.util.ThisRobotTable;
import frc.robot.util.coprocessortable.VelocityCommand;

public class DriveToClimberBar extends CommandBase {
  public static enum DriveToClimberBarState {
    IDLE, CLIMBER_PREP_WAYPOINT, PREP_LINEUP, LINEUP_TO_BAR, FINISHED
  }

  private final Navigation m_nav;
  private final Drive m_drive;
  private final Sensors m_sensors;
  private final ThisRobotTable m_robot_table;
  private WaypointsPlan m_plan;
  private final String m_waypointName = "<team>_climber_prep";
  private long m_is_finished_timeout = 0;
  private double m_barReachedDistanceMeters = 0.0;
  private DriveToClimberBarState m_state = DriveToClimberBarState.IDLE;

  /** Creates a new DriveToWaypoint. */
  public DriveToClimberBar(Navigation nav, ThisRobotTable robot_table, Drive drive, Sensors sensors, long is_finished_timeout, double barReachedDistanceMeters) {
    m_nav = nav;
    m_drive = drive;
    m_sensors = sensors;
    m_robot_table = robot_table;
    m_is_finished_timeout = is_finished_timeout;
    m_plan = new WaypointsPlan(m_nav.getCoprocessorTable());
    m_plan.addWaypoint(new Waypoint(m_waypointName));
    m_barReachedDistanceMeters = barReachedDistanceMeters;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(nav);
    addRequirements(drive);
  }
  public DriveToClimberBar(Navigation nav, ThisRobotTable robot_table, Drive drive, Sensors sensors, double barReachedDistanceMeters) {
    this(nav, robot_table, drive, sensors, 15_000_000, barReachedDistanceMeters);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_nav.setWaypointsPlan(m_plan, m_is_finished_timeout);
    m_sensors.setCameraTilterAngle(Constants.CAMERA_TILT_DOWN_ANGLE);
    m_state = DriveToClimberBarState.CLIMBER_PREP_WAYPOINT;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_state) {
      case CLIMBER_PREP_WAYPOINT:
        VelocityCommand command = m_nav.getAutoCommand();
        if (Objects.isNull(command)) {
          return;
        }
        m_drive.drive(command);
        m_sensors.setCameraTilterAngle(m_robot_table.getCameraTiltCommand());
        if (m_nav.isRosAutoFailed()) {
          System.out.println("Drive to waypoint failed. Aborting bar line up");
          m_state = DriveToClimberBarState.FINISHED;
        }
        else if (m_nav.isRosAutoFinished()) {
          m_state = DriveToClimberBarState.PREP_LINEUP;
        }
        break;
      case PREP_LINEUP:
        m_nav.cancelAutoGoal();
        m_sensors.setCameraTilterAngle(Constants.CAMERA_TILT_UP_ANGLE);
        m_drive.stop();
        m_state = DriveToClimberBarState.LINEUP_TO_BAR;
        break;
      case LINEUP_TO_BAR:
        if (!m_robot_table.isBarValid() || m_robot_table.getBarCount() <= 0) {
          break;
        }
        double barAngle = m_robot_table.getBarAngle();
        double barDistance = m_robot_table.getBarDistance();

        // TODO: add drive commands and PID controller

        if (Math.abs(barDistance) < m_barReachedDistanceMeters) {
          m_state = DriveToClimberBarState.FINISHED;
        }
        
      case FINISHED:
        break;
      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_nav.cancelAutoGoal();
    m_sensors.setCameraTilterAngle(Constants.CAMERA_TILT_DOWN_ANGLE);
    m_drive.stop();
    m_state = DriveToClimberBarState.IDLE;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_state == DriveToClimberBarState.FINISHED;
  }
}
