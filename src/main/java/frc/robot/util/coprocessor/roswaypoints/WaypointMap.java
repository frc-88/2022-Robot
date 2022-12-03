package frc.robot.util.coprocessor.roswaypoints;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.util.coprocessor.CoprocessorBase;
import frc.robot.util.coprocessor.Helpers;

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

    public Pose2d getWaypoint(String waypointName) {
        waypointName = Helpers.parseName(waypointName);
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

    public Pose2d getPoseRelativeToWaypoint(String waypointName, Pose2d relativePose) {
        Pose2d waypoint = getWaypoint(waypointName);
        if (!isPoseValid(waypoint)) {
            return waypoint;
        }
        
        return waypoint.transformBy(new Transform2d(relativePose, new Pose2d()));
    }
}
