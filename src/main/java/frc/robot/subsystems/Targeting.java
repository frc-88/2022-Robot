// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.drive.DriveUtils;
import frc.robot.util.sensors.Limelight;

public class Targeting extends SubsystemBase {
  private final Limelight m_limelight;
  private final Navigation m_nav;
  private final Turret m_turret;
  private double m_target_angle = 0.0;
  private double m_target_dist = 0.0;
  private boolean m_has_target = false;

  private enum TARGETING_MODE {
    LIMELIGHT_ONLY,
    WAYPOINT_ONLY,
    COMBO;
  }

  /** Creates a new Targeting. */
  public Targeting(Limelight limelight, Navigation nav, Turret turret) {
    m_limelight = limelight;
    m_nav = nav;
    m_turret = turret;
  }

  private static final double LIMELIGHT_WAYPOINT_AGREEMENT_ANGLE_DEGREES = 27.0;
  private static final double LIMELIGHT_WAYPOINT_AGREEMENT_DIST_INCHES = 3.0;
  private final TARGETING_MODE targeting_mode = TARGETING_MODE.WAYPOINT_ONLY;
  // private final TARGETING_MODE targeting_mode = TARGETING_MODE.LIMELIGHT_ONLY;
  // private final TARGETING_MODE targeting_mode = TARGETING_MODE.COMBO;

  private Pair<Double, Double> getLimelightTarget(Limelight limelight, Turret turret) {
      double angle = Double.NaN;
      if (limelight.onTarget()) {
          // keep on same target
          angle = turret.getFacing();
      } else if (limelight.hasTarget()) {
          // if we have a target, track it
          angle = turret.getFacing() - limelight.getTurretOffset();
      }
      double distance = limelight.getTargetDistance();
      if (distance <= 0.0) {
          distance = Double.NaN;
      }
      else {
          distance += Constants.FIELD_UPPER_HUB_RADIUS;
      }
      return new Pair<Double, Double>(distance, angle);
  }

  private Pair<Double, Double> getWaypointTarget(Navigation nav) {
    if (nav.isShooterTargetValid()) {
      double baseTargetAngle = Math.toDegrees(nav.getShooterAngle());
      double adder = DriveUtils.mod(baseTargetAngle + 180., 360) - DriveUtils.mod(m_turret.getFacing() + 180., 360);
      double finalAngle = m_turret.getFacing() + adder;
      return new Pair<Double, Double>(
        Units.metersToInches(nav.getShooterDistance()),
        finalAngle);
    }
    else {
      return new Pair<Double, Double>(Double.NaN, Double.NaN);
    }
  }

  public double getShooterDistance() {
    // Shooter/Turret target in inches
    return m_target_dist;
  }

  public double getTurretAngle() {
    // Shooter/Turret angle in degrees
    return m_target_angle;
  }

  public boolean isTracking() {
    return m_turret.isTracking();
  }

  public void disableTurret() {
    m_turret.goToDefaultFacing();
    m_limelight.ledOff();
  }

  public void enableTurret() {
    m_limelight.ledOn();
    //   if (targeting_mode != TARGETING_MODE.WAYPOINT_ONLY) {
    //   m_limelight.ledOn();
    // }
  }

  @Override
  public void periodic() {
    double target_angle = m_turret.getDefaultFacing();
    double target_dist = Double.NaN;
    boolean has_target = true;

    double limelight_target_dist = Double.NaN;
    double limelight_target_angle = Double.NaN;
    double waypoint_target_dist = Double.NaN;
    double waypoint_target_angle = Double.NaN;
    Pair<Double, Double> limelight_target;
    Pair<Double, Double> waypoint_target;

    // both limelight_target and waypoint_target are in the format: distance (inches), angle (degrees)
    
    switch (targeting_mode) {
      case LIMELIGHT_ONLY:
        limelight_target = getLimelightTarget(m_limelight, m_turret);
        limelight_target_dist = limelight_target.getFirst();
        limelight_target_angle = limelight_target.getSecond();
        break;
      case WAYPOINT_ONLY:
        waypoint_target = getWaypointTarget(m_nav);
        waypoint_target_dist = waypoint_target.getFirst();
        waypoint_target_angle = waypoint_target.getSecond();
        break;
      case COMBO:
        limelight_target = getLimelightTarget(m_limelight, m_turret);
        limelight_target_dist = limelight_target.getFirst();
        limelight_target_angle = limelight_target.getSecond();
        
        waypoint_target = getWaypointTarget(m_nav);
        waypoint_target_dist = waypoint_target.getFirst();
        waypoint_target_angle = waypoint_target.getSecond();
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
        SmartDashboard.putNumber("Turret:Limelight-waypoint delta (degrees)", limelight_global_delta);
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

    SmartDashboard.putNumber("Turret:Waypoint target angle (degrees)", waypoint_target_angle);
    SmartDashboard.putNumber("Turret:Limelight target angle (degrees)", limelight_target_angle);
    SmartDashboard.putNumber("Turret:Track target angle (degrees)", target_angle);

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

    SmartDashboard.putNumber("Turret:Waypoint target dist (inches)", waypoint_target_dist);
    SmartDashboard.putNumber("Turret:Limelight target dist (inches)", limelight_target_dist);
    SmartDashboard.putNumber("Turret:Track target dist (inches)", target_dist);

    m_target_dist = target_dist;
    m_target_angle = target_angle;
    m_has_target = has_target;
  }
}