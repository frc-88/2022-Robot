// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.roswaypoints.GoalStatus;
import frc.robot.util.roswaypoints.WaypointMap;
import frc.robot.util.roswaypoints.WaypointsPlan;
import frc.robot.util.coprocessortable.CoprocessorTable;
import frc.robot.util.coprocessortable.VelocityCommand;

public class Navigation extends SubsystemBase {
  private final WaypointMap m_waypointMap = new WaypointMap();
  private final CoprocessorTable m_coprocessor;

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
  public Navigation(CoprocessorTable coprocessor) {
    m_coprocessor = coprocessor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public WaypointsPlan makeEmptyWaypointPlan() {
    return new WaypointsPlan(m_coprocessor);
  }

  public Pose2d getRobotPose() {
    return m_coprocessor.getGlobalPose();
  }

  public Pose2d getCenterWaypoint() {
    return m_waypointMap.getWaypoint("center");
  }

  public boolean isPoseValid(Pose2d pose) {
    return m_waypointMap.isPoseValid(pose);
  }

  private Pose2d calculateNearestRingPose(double ringRadius) {
    Pose2d pr = getRobotPose();
    Pose2d pc = getCenterWaypoint();
    double slope = (pr.getY() - pc.getY()) / (pr.getX() - pc.getX());
    double x1 = pc.getX();
    double y1 = pc.getY();
    double numerator_1 = Math.sqrt((slope * slope + 1.0) * ringRadius - Math.pow(slope * x1 - y1, 2));
    double numerator_2 = slope * (slope * x1 - y1);
    double denominator = slope * slope + 1.0;

    double ring_x1 = (-numerator_1 + numerator_2) / denominator;
    double ring_y1 = slope * (ring_x1 - x1) + y1;
    double ring_x2 = (numerator_1 + numerator_2) / denominator;
    double ring_y2 = slope * (ring_x2 - x1) + y1;

    double ring_1_dist = getDistance(pr.getX(), pr.getY(), ring_x1, ring_y1);
    double ring_2_dist = getDistance(pr.getX(), pr.getY(), ring_x2, ring_y2);
    double centerHeading = 0.0;
    if (ring_1_dist < ring_2_dist) {
      centerHeading = getHeading(pc.getX(), pc.getY(), ring_x1, ring_y1);
      return new Pose2d(ring_x1, ring_y1, new Rotation2d(centerHeading));
    } else {
      centerHeading = getHeading(pc.getX(), pc.getY(), ring_x2, ring_y2);
      return new Pose2d(ring_x2, ring_y2, new Rotation2d(centerHeading));
    }
  }

  public Pose2d calculateNearestShootingZonePose(double innerRadius, double outerRadius) {
    Pose2d pr = getRobotPose();
    Pose2d pc = getCenterWaypoint();
    double robot_dist = getDistance(pc.getX(), pc.getY(), pr.getX(), pr.getY());
    if (Math.abs(robot_dist - innerRadius) > Math.abs(robot_dist - outerRadius)) {
      return calculateNearestRingPose(innerRadius);
    } else {
      return calculateNearestRingPose(outerRadius);
    }
  }

  private double getHeading(double x1, double y1, double x2, double y2) {
    double x = x2 - x1;
    double y = y2 - y1;
    return Math.atan2(y, x);
  }

  private double getDistance(double x1, double y1, double x2, double y2) {
    double x = x2 - x1;
    double y = y2 - y1;
    return Math.sqrt(x * x + y * y);
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
