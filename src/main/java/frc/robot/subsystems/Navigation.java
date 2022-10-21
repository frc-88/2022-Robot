// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Objects;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.coprocessor.VelocityCommand;
import frc.robot.util.coprocessor.CoprocessorBase;
import frc.robot.util.roswaypoints.GoalStatus;
import frc.robot.util.roswaypoints.WaypointMap;
import frc.robot.util.roswaypoints.WaypointsPlan;

public class Navigation extends SubsystemBase {
  private final WaypointMap m_waypointMap;
  private final CoprocessorBase m_coprocessor;
  public static final String CENTER_WAYPOINT_NAME = "center";

  public static enum RosAutoState {
    SEND_PLAN, WAIT_FOR_RUNNING, WAIT_FOR_FINISHED, FINISHED
  }

  private RosAutoState m_ros_auto_state = RosAutoState.FINISHED;

  private WaypointsPlan m_plan;
  private long m_is_running_timer = 0;
  private long m_is_finished_timer = 0;
  private long m_is_running_timeout = 1_000_000;
  private long m_is_finished_timeout = 0;

  /** Creates a new NavigationSubsystem. */
  public Navigation(CoprocessorBase coprocessor) {
    m_coprocessor = coprocessor;
    m_waypointMap = new WaypointMap(m_coprocessor);
  }

  @Override
  public void periodic() {

  }

  public CoprocessorBase getCoprocessorBase() {
    return m_coprocessor;
  }

  public Set<String> getWaypointNames() {
    return m_waypointMap.getWaypointNames();
  }
  public boolean doesWaypointExist(String waypointName) {
    return m_waypointMap.doesWaypointExist(waypointName);
  }

  public boolean isConnected() {
    return m_coprocessor.isConnected();
  }
  public WaypointsPlan makeEmptyWaypointPlan() {
    return new WaypointsPlan(m_coprocessor);
  }

  public Pose2d getRobotPose() {
    return m_coprocessor.getGlobalPose();
  }

  public Pose2d getWaypoint(String name) {
    return m_waypointMap.getWaypoint(name);
  }

  public Pose2d getCenterWaypoint() {
    return getWaypoint(CENTER_WAYPOINT_NAME);
  }

  public void setPoseEstimate(Pose2d pose) {
    m_coprocessor.setPoseEstimate(pose);
  }

  public boolean isPoseValid(Pose2d pose) {
    return m_waypointMap.isPoseValid(pose);
  }

  private long getTime() {
    return RobotController.getFPGATime();
  }

  public void setWaypointsPlan(WaypointsPlan plan, long is_finished_timeout) {
    if (m_ros_auto_state == RosAutoState.FINISHED) {
      m_ros_auto_state = RosAutoState.SEND_PLAN;
      m_plan = plan;
      m_is_finished_timeout = is_finished_timeout;
      return;
    }
    System.out.println("Plan is still running! Cancel it first. Not setting plan");
  }

  public void cancelAutoGoal() {
    m_coprocessor.cancelGoal();
    m_ros_auto_state = RosAutoState.FINISHED;
  }

  public RosAutoState getRosAutoState() {
    return m_ros_auto_state;
  }

  public VelocityCommand getAutoCommand() {
    VelocityCommand command = null;
    if (Objects.isNull(m_plan)) {
      m_ros_auto_state = RosAutoState.FINISHED;
      return command;
    }
    switch (m_ros_auto_state) {
      case SEND_PLAN:
        m_plan.sendWaypoints();
        m_ros_auto_state = RosAutoState.WAIT_FOR_RUNNING;
        m_is_running_timer = getTime();
        break;
      case WAIT_FOR_RUNNING:
        if (m_coprocessor.getGoalStatus() == GoalStatus.RUNNING) {
          m_ros_auto_state = RosAutoState.WAIT_FOR_FINISHED;
          m_is_finished_timer = getTime();
        }
        if (getTime() - m_is_running_timer > m_is_running_timeout) {
          System.out.println("Timeout exceeded while waiting for goal to signal running");
          m_ros_auto_state = RosAutoState.FINISHED;
        }
        break;
      case WAIT_FOR_FINISHED:
        if (m_is_finished_timeout > 0 && getTime() - m_is_finished_timer > m_is_finished_timeout) {
          System.out.println("Timeout exceeded while waiting for goal to signal running");
          m_ros_auto_state = RosAutoState.FINISHED;
        }
        switch (m_coprocessor.getGoalStatus()) {
          case RUNNING:
            break;
          case INVALID:
          case IDLE:
          case FAILED:
            System.out.println("Coprocessor entered into an aborted state. Cancelling goal.");
          case FINISHED:
            m_ros_auto_state = RosAutoState.FINISHED;
            break;
        }
        if (m_coprocessor.isCommandActive()) {
          command = m_coprocessor.getCommand();
        }
        break;
      case FINISHED:
        break;
    }
    return command;
  }
}