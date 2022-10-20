// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.ThisRobotTable;
import frc.robot.util.drive.DriveUtils;
import frc.robot.util.preferenceconstants.BooleanPreferenceConstant;
import frc.robot.util.sensors.Limelight;

public class Targeting extends SubsystemBase {
  private final Limelight m_limelight;
  private final Turret m_turret;
  private final ThisRobotTable m_ros_interface;
  private final SwerveDrive m_drive;
  private final Shooter m_shooter;

  private double m_target_angle = 0.0;
  private double m_target_dist = 0.0;
  private double m_shot_probability = 1.0;
  private boolean m_has_target = false;
  
  private BooleanPreferenceConstant p_limelightMovingTargetMode = new BooleanPreferenceConstant("LL Moving Shot", false);

  private long m_resetToLimelightCooldownTimer = 0;
  private long m_resetToLimelightCooldown = 1_000_000;

  private enum TARGETING_MODE {
    LIMELIGHT_ONLY,
    WAYPOINT_ONLY,
    COMBO;
  }

  private boolean m_enableDefault = false;
  private double m_defaultDistance = 0;
  private double m_defaultAngle = 0;

  /** Creates a new Targeting. */
  public Targeting(Limelight limelight, ThisRobotTable ros_interface, Turret turret, SwerveDrive drive, Shooter shooter) {
    m_limelight = limelight;
    m_turret = turret;
    m_ros_interface = ros_interface;
    m_drive = drive;
    m_shooter = shooter;
  }

  private static final double LIMELIGHT_WAYPOINT_AGREEMENT_ANGLE_DEGREES = 27.0;
  private static final double LIMELIGHT_WAYPOINT_AGREEMENT_DIST_INCHES = 30.0;
  private TARGETING_MODE DEFAULT_MODE = TARGETING_MODE.WAYPOINT_ONLY;
  private TARGETING_MODE targeting_mode = DEFAULT_MODE;

  private Pair<Double, Double> getLimelightTarget() {
    boolean llMoving = p_limelightMovingTargetMode.getValue();
    double angle = Double.NaN;
    double distance = m_limelight.getTargetDistance();
    double turretFacing = m_turret.getFacing();

    if (llMoving && m_limelight.hasTarget()) {
      distance = m_limelight.calcMovingDistance(m_drive.getStraightSpeed(), turretFacing, true);
      angle = turretFacing - m_limelight.getTurretOffset() 
        - m_limelight.calcMovingTurretOffset(m_drive.getStraightSpeed(), turretFacing, distance, true);
    } else if (m_limelight.onTarget()) {
        // keep on same target
        angle = turretFacing;
      } else if (m_limelight.hasTarget()) {
        // if we have a target, track it
        angle = turretFacing - m_limelight.getTurretOffset();
      }

    if (distance <= 0.0) {
      distance = Double.NaN;
    }

    // Leave this on for ROS logging
    SmartDashboard.putNumber("Limelight Target Angle", angle);
    SmartDashboard.putNumber("Limelight Target Distance", distance);

    m_shooter.registerLimelightTarget(distance, angle);

    return new Pair<Double, Double>(distance, angle);
  }

  private Pair<Double, Double> getWaypointTarget() {
    if (m_ros_interface.isShooterTargetValid()) {
      double baseTargetAngle = Math.toDegrees(m_ros_interface.getShooterAngle());
      double adder = DriveUtils.mod(baseTargetAngle + 180., 360) - DriveUtils.mod(m_turret.getFacing() + 180., 360);
      double finalAngle = m_turret.getFacing() + adder;

      m_shooter.registerROSTarget(Units.metersToInches(m_ros_interface.getShooterDistance()), finalAngle);

      return new Pair<Double, Double>(
        Units.metersToInches(m_ros_interface.getShooterDistance()),
        finalAngle);
    }
    else {
      m_shooter.registerROSTarget(0, 0);

      return new Pair<Double, Double>(Double.NaN, Double.NaN);
    }
  }

  public void enableDefault(double distance, double angle) {
    m_enableDefault = true;
    m_defaultDistance = distance;
    m_defaultAngle = angle;
  }

  public void disableDefault() {
    m_enableDefault = false;
  }

  public double getShooterDistance() {
    // Shooter/Turret target in inches
    if (m_enableDefault && DriverStation.isAutonomous() && (Math.abs(m_target_dist - m_defaultDistance) > 24. || Math.abs(DriveUtils.mod(m_target_dist, 360) - DriveUtils.mod(m_defaultDistance, 360)) > 20)) {
      System.out.println("Using default (" + m_defaultDistance + ", " + m_defaultAngle + ") instead of (" + m_target_dist + ", " + m_target_angle + ")");
      return m_defaultDistance;
    }
    return m_target_dist;
  }

  public double getTurretAngle() {
    // Shooter/Turret angle in degrees
    if (m_enableDefault && DriverStation.isAutonomous() && (Math.abs(m_target_dist - m_defaultDistance) > 24. || Math.abs(DriveUtils.mod(m_target_dist, 360) - DriveUtils.mod(m_defaultDistance, 360)) > 20)) {
      return m_defaultAngle;
    }
    return m_target_angle;
  }

  public boolean isTracking() {
    return m_turret.isTracking();
  }

  public boolean hasTarget() {
    return m_has_target;
  }

  public void disableTurret() {
    m_turret.goToDefaultFacing();
    m_limelight.ledOff();
  }

  public void enableTurret() {
    m_limelight.ledOn();
    // if (targeting_mode != TARGETING_MODE.WAYPOINT_ONLY) {
    //   m_limelight.ledOn();
    // }
    // else {
    //   m_limelight.ledOff();
    // }
  }

  public void setTargetingMode(TARGETING_MODE mode) {
    targeting_mode = mode;
  }

  public void setModeToLimelight() {
    targeting_mode = TARGETING_MODE.LIMELIGHT_ONLY;
  }

  public void setModeToWaypoint() {
    targeting_mode = TARGETING_MODE.WAYPOINT_ONLY;
  }

  public void setModeToCombo() {
    targeting_mode = TARGETING_MODE.COMBO;
  }

  public void setModeToDefault() {
    targeting_mode = DEFAULT_MODE;
  }

  /**
   * Like periodic(), but runs before commands
   */
  public void firstPeriodic() {
    double target_angle = m_turret.getDefaultFacing();
    double target_dist = Double.NaN;
    boolean has_target = true;

    double limelight_target_dist = Double.NaN;
    double limelight_target_angle = Double.NaN;
    double waypoint_target_dist = Double.NaN;
    double waypoint_target_angle = Double.NaN;
    Pair<Double, Double> limelight_target;
    Pair<Double, Double> waypoint_target;

    limelight_target = getLimelightTarget();
    limelight_target_dist = limelight_target.getFirst();
    limelight_target_angle = limelight_target.getSecond();
    
    waypoint_target = getWaypointTarget();
    waypoint_target_dist = waypoint_target.getFirst();
    waypoint_target_angle = waypoint_target.getSecond();


    // both limelight_target and waypoint_target are in the format: distance (inches), angle (degrees)
    
    switch (targeting_mode) {
      case LIMELIGHT_ONLY:
        // limelight_target = getLimelightTarget();
        // limelight_target_dist = limelight_target.getFirst();
        // limelight_target_angle = limelight_target.getSecond();
        waypoint_target_dist = Double.NaN;
        waypoint_target_angle = Double.NaN;

        m_shot_probability = 1.;
        break;
      case WAYPOINT_ONLY:
        // waypoint_target = getWaypointTarget();
        // waypoint_target_dist = waypoint_target.getFirst();
        // waypoint_target_angle = waypoint_target.getSecond();
        limelight_target_dist = Double.NaN;
        limelight_target_angle = Double.NaN;

        m_shot_probability = m_ros_interface.getShooterProbability();
        break;
      case COMBO:
        // limelight_target = getLimelightTarget();
        // limelight_target_dist = limelight_target.getFirst();
        // limelight_target_angle = limelight_target.getSecond();
        
        // waypoint_target = getWaypointTarget();
        // waypoint_target_dist = waypoint_target.getFirst();
        // waypoint_target_angle = waypoint_target.getSecond();

        m_shot_probability = m_ros_interface.getShooterProbability();
        break;
      default:
        break;
    }

    if (Double.isNaN(limelight_target_angle) && Double.isNaN(waypoint_target_angle)) {
        // If neither the limelight or waypoint have valid targets, tell the turret to return to default
        target_angle = m_turret.getDefaultFacing();
        target_dist = 0.0;
        has_target = false;
    }
    else if (Double.isNaN(limelight_target_angle)) {
        // If the limelight doesn't have a target, use the waypoint angle and distance
        target_angle = waypoint_target_angle;
        target_dist = waypoint_target_dist;
        has_target = true;
    }
    else if (Double.isNaN(waypoint_target_angle)) {
        // If the waypoint doesn't have a target, use the limelight angle and distance
        target_angle = limelight_target_angle;
        target_dist = limelight_target_dist;
        has_target = true;
    }
    else {
        // If both the limelight and waypoint have a target,
        double limelight_global_delta = Math.abs((limelight_target_angle % 360) - waypoint_target_angle);
        if (RobotContainer.isPublishingEnabled()) {
          SmartDashboard.putNumber("Targeting:Limelight-waypoint delta (degrees)", limelight_global_delta);
        }
        if (limelight_global_delta <= LIMELIGHT_WAYPOINT_AGREEMENT_ANGLE_DEGREES) {
            // if the limelight and waypoint target are within a threshold of agreement, use the limelight's target angle
            target_angle = limelight_target_angle;
        } else {
            // if the limelight and waypoint target are not within a threshold of agreement, use the waypoint's target value
            target_angle = waypoint_target_angle;
            target_dist = waypoint_target_dist;
        }
        has_target = true;
    }

    if (RobotContainer.isPublishingEnabled()) {
      SmartDashboard.putNumber("Targeting:Waypoint target angle (degrees)", waypoint_target_angle);
      SmartDashboard.putNumber("Targeting:Limelight target angle (degrees)", limelight_target_angle);
      SmartDashboard.putNumber("Targeting:Track target angle (degrees)", target_angle);
    }

    if (Double.isNaN(target_dist)) {
        // If the target distance hasn't been set yet,
        if (Double.isNaN(limelight_target_dist) && Double.isNaN(waypoint_target_dist)) {
            // If neither the limelight or waypoint have valid targets, tell the shooter to shoot blindly
            target_dist = 0.0;
            has_target = false;
        }
        else if (Double.isNaN(limelight_target_dist)) {
            // If the limelight doesn't have a target, use the waypoint distance to shoot
            target_dist = waypoint_target_dist;
        }
        else if (Double.isNaN(waypoint_target_dist)) {
            // If the waypoint doesn't have a target, use the limelight distance to shoot
            target_dist = limelight_target_dist;
        }
        else {
            // If both the limelight and waypoint have a target,
            if (Math.abs(limelight_target_dist - waypoint_target_dist) <= LIMELIGHT_WAYPOINT_AGREEMENT_DIST_INCHES) {
                // if the limelight and waypoint target are within a threshold of agreement, use the limelight's target distance
                target_dist = limelight_target_dist;
            }
            else {
                // if the limelight and waypoint target are not within a threshold of agreement, use the waypoint's distance
                target_dist = waypoint_target_dist;
            }
        }
    }

    if (RobotContainer.isPublishingEnabled()) {
      SmartDashboard.putNumber("Targeting:Waypoint target dist (inches)", waypoint_target_dist);
      SmartDashboard.putNumber("Targeting:Limelight target dist (inches)", limelight_target_dist);
      SmartDashboard.putNumber("Targeting:Track target dist (inches)", target_dist);
    }

    m_target_dist = target_dist;
    m_target_angle = target_angle;
    m_has_target = has_target;

    if (targeting_mode == TARGETING_MODE.LIMELIGHT_ONLY) {
      // Tell ROS to reset to the limelight + odometry estimated position
      long currentTime = RobotController.getFPGATime();
      if (currentTime - m_resetToLimelightCooldownTimer > m_resetToLimelightCooldown) {
        m_ros_interface.resetPoseToLimelight();
        m_resetToLimelightCooldownTimer = currentTime;
      }
    }
    SmartDashboard.putBoolean("Targeting:Has Target", m_has_target);
  }
}
