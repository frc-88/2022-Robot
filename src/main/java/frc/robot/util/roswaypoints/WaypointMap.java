package frc.robot.util.roswaypoints;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.coprocessor.CoprocessorBase;

public class WaypointMap {
    private CoprocessorBase m_coprocessor;
    
    public WaypointMap(CoprocessorBase coprocessor) {
        m_coprocessor = coprocessor;
    }

    public Set<String> getWaypointNames() {
        return m_coprocessor.getWaypointNames();
    }
    public boolean doesWaypointExist(String waypointName) {
        return m_coprocessor.doesWaypointExist(waypointName);
    }

    public static String getTeamColorName() {
        if (DriverStation.getAlliance() == Alliance.Red) {
            return "red";
        }
        else {
            return "blue";
        }
    }

    public static String parseWaypointName(String waypointName) {
        return waypointName.replaceAll("<team>", getTeamColorName());
    }

    public Pose2d getWaypoint(String waypointName) {
        waypointName = parseWaypointName(waypointName);
        System.out.println("Getting waypoint " + waypointName);
        if (doesWaypointExist(waypointName)) {
            return m_coprocessor.getWaypoint(waypointName);
        }
        else {
            return new Pose2d(Double.NaN, Double.NaN, new Rotation2d(Double.NaN));
        }
    }
    public boolean isPoseValid(Pose2d pose) {
        return !Double.isNaN(pose.getX()) && !Double.isNaN(pose.getY()) && !Double.isNaN(pose.getRotation().getRadians());
    }
}