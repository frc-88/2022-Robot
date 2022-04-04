package frc.robot.util.roswaypoints;

import java.util.Objects;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.coprocessortable.CoprocessorTable;

public class WaypointMap {
    private final NetworkTable m_table;
    private NewWaypointInterface m_callback;
    
    public WaypointMap(CoprocessorTable coprocessor, NewWaypointInterface callback) {
        m_table = coprocessor.getWaypointsTable();
        m_callback = callback;
        m_table.addSubTableListener((parent, name, table) -> {newWaypointCallback(name);}, true);
    }

    public WaypointMap(CoprocessorTable coprocessor) {
        m_table = coprocessor.getWaypointsTable();
    }

    private void newWaypointCallback(String name) {
        if (Objects.nonNull(m_callback)) {
            m_callback.newWaypointCallback(this, name);
        }
    }
    public Set<String> getWaypointNames() {
        return m_table.getSubTables();
    }
    public boolean doesWaypointExist(String waypointName) {
        return m_table.containsSubTable(waypointName);
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
        if (doesWaypointExist(waypointName)) {
            double x = m_table.getSubTable(waypointName).getEntry("x").getDouble(Double.NaN);
            double y = m_table.getSubTable(waypointName).getEntry("y").getDouble(Double.NaN);
            double theta = m_table.getSubTable(waypointName).getEntry("theta").getDouble(Double.NaN);
            return new Pose2d(x, y, new Rotation2d(theta));
        }
        else {
            return new Pose2d(Double.NaN, Double.NaN, new Rotation2d(Double.NaN));
        }
    }
    public boolean isPoseValid(Pose2d pose) {
        return !Double.isNaN(pose.getX()) && !Double.isNaN(pose.getY()) && !Double.isNaN(pose.getRotation().getRadians());
    }
}
