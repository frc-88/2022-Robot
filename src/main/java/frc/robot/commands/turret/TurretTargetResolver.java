package frc.robot.commands.turret;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Turret;
import frc.robot.util.sensors.Limelight;

public class TurretTargetResolver {
    private static final double LIMELIGHT_WAYPOINT_AGREEMENT_ANGLE_DEGREES = 45.0;
    private static final double LIMELIGHT_WAYPOINT_AGREEMENT_DIST_METERS = 3.0;

    private static Pair<Double, Double> getLimelightTarget(Limelight limelight, Turret turret) {
        double angle = Double.NaN;
        if (limelight.onTarget()) {
            // keep on same target
            angle = turret.getFacing();
        } else if (limelight.hasTarget()) {
            // if we have a target, track it
            angle = turret.getFacing() - limelight.calcTurretOffset();
        }
        double distance = limelight.calcDistanceToTarget();
        if (distance == 0.0) {
            distance = Double.NaN;
        }
        else {
            distance += Constants.FIELD_UPPER_HUB_RADIUS;
      
            distance = Units.inchesToMeters(distance);
        }
        return new Pair<Double, Double>(distance, angle);
    }

    private static Pair<Double, Double> getWaypointTarget(Navigation nav, String waypointName, Turret turret) {
        Pose2d target_map = nav.getWaypoint(waypointName);
        SmartDashboard.putBoolean("Turret:Is target valid", nav.isPoseValid(target_map));
        if (!nav.isPoseValid(target_map)) {
            return new Pair<Double, Double>(Double.NaN, Double.NaN);
        }
        Pose2d robot_pose = nav.getRobotPose();
        Translation2d robot_relative_to_target = robot_pose.getTranslation().minus(target_map.getTranslation());
        double tx = robot_relative_to_target.getX();
        double ty = robot_relative_to_target.getY();

        double target_global_angle = Math.atan2(ty, tx);
        double waypoint_target_angle = target_global_angle - robot_pose.getRotation().getRadians();
        waypoint_target_angle = Math.toDegrees(waypoint_target_angle);
        waypoint_target_angle = waypoint_target_angle % 360.0;
        double waypoint_target_distance = Math.sqrt(tx * tx + ty * ty);
        return new Pair<Double, Double>(waypoint_target_distance, waypoint_target_angle);
    }

    public static Pair<Double, Double> getTurretTarget(Navigation nav, String waypointName, Limelight limelight, Turret turret) {
        double turret_target_angle = 0.0;
        double turret_target_dist = Double.NaN;
        Pair<Double, Double> limelight_target = getLimelightTarget(limelight, turret);
        Pair<Double, Double> waypoint_target = getWaypointTarget(nav, waypointName, turret);
        
        double limelight_target_dist = limelight_target.getFirst();
        double limelight_target_angle = limelight_target.getSecond();
        
        double waypoint_target_dist = waypoint_target.getFirst();
        double waypoint_target_angle = waypoint_target.getSecond();

        if (Double.isNaN(limelight_target_angle) && Double.isNaN(waypoint_target_angle)) {
            turret_target_angle = 0.0;
        }
        else if (Double.isNaN(limelight_target_angle)) {
            turret_target_angle = waypoint_target_angle;
        }
        else if (Double.isNaN(waypoint_target_angle)) {
            turret_target_angle = limelight_target_angle;
        }
        else {
            double limelight_global_delta = Math.abs((limelight_target_angle % 360) - waypoint_target_angle);
            if (LIMELIGHT_WAYPOINT_AGREEMENT_ANGLE_DEGREES == 0.0 || limelight_global_delta > 45.0) {
                turret_target_angle = waypoint_target_angle;
                turret_target_dist = waypoint_target_dist;  // use waypoint distance if the limelight is out of agreement
            } else {
                turret_target_angle = limelight_target_angle;
            }
        }

        SmartDashboard.putNumber("Turret:Waypoint target angle (degrees)", waypoint_target_angle);
        SmartDashboard.putNumber("Turret:Limelight target angle (degrees)", limelight_target_angle);
        SmartDashboard.putNumber("Turret:Track target angle (degrees)", turret_target_angle);

        if (Double.isNaN(turret_target_dist)) {
            if (Double.isNaN(limelight_target_dist) && Double.isNaN(waypoint_target_dist)) {
                turret_target_dist = 0.0;
            }
            else if (Double.isNaN(limelight_target_dist)) {
                turret_target_dist = waypoint_target_dist;
            }
            else if (Double.isNaN(waypoint_target_dist)) {
                turret_target_dist = limelight_target_dist;
            }
            else {
                if (Math.abs(limelight_target_dist - waypoint_target_dist) > LIMELIGHT_WAYPOINT_AGREEMENT_DIST_METERS) {
                    turret_target_dist = waypoint_target_dist;
                }
                else {
                    turret_target_dist = limelight_target_dist;
                }
            }
        }

        SmartDashboard.putNumber("Turret:Waypoint target dist (meters)", waypoint_target_dist);
        SmartDashboard.putNumber("Turret:Limelight target dist (meters)", limelight_target_dist);
        SmartDashboard.putNumber("Turret:Track target dist (meters)", turret_target_dist);

        return new Pair<Double, Double>(turret_target_dist, turret_target_angle);
    }
}
