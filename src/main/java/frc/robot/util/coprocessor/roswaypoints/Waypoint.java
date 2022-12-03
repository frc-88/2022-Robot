package frc.robot.util.coprocessor.roswaypoints;

import edu.wpi.first.math.geometry.Pose2d;

public class Waypoint {
    public static final double DEFAULT_INTERMEDIATE_TOLERANCE = 0.2;
    public String waypoint_name = "";
    public Pose2d pose = new Pose2d();
    public boolean is_continuous = false;
    public boolean ignore_orientation = true;
    public double intermediate_tolerance = DEFAULT_INTERMEDIATE_TOLERANCE;
    public boolean ignore_obstacles = false;
    public boolean ignore_walls = false;
    public double timeout = 0.0;

    // -----
    // Named waypoint constructors
    // -----
    public Waypoint(String waypoint_name, boolean is_continuous, boolean ignore_orientation, double intermediate_tolerance, boolean ignore_obstacles, boolean ignore_walls, double timeout)
    {
        this.waypoint_name = waypoint_name;
        this.is_continuous = is_continuous;
        this.ignore_orientation = ignore_orientation;
        this.intermediate_tolerance = intermediate_tolerance;
        this.ignore_obstacles = ignore_obstacles;
        this.ignore_walls = ignore_walls;
        this.timeout = timeout;
    }

    public Waypoint(String waypoint_name, boolean is_continuous) {
        this(waypoint_name, is_continuous, true, DEFAULT_INTERMEDIATE_TOLERANCE, false, false, 0.0);
    }

    public Waypoint(String waypoint_name) {
        this(waypoint_name, false, true, DEFAULT_INTERMEDIATE_TOLERANCE, false, false, 0.0);
    }


    // -----
    // Pose waypoint constructors
    // -----
    public Waypoint(Pose2d pose, boolean is_continuous, boolean ignore_orientation, double intermediate_tolerance, boolean ignore_obstacles, boolean ignore_walls, double timeout)
    {
        this.pose = pose;
        this.is_continuous = is_continuous;
        this.ignore_orientation = ignore_orientation;
        this.intermediate_tolerance = intermediate_tolerance;
        this.ignore_obstacles = ignore_obstacles;
        this.ignore_walls = ignore_walls;
        this.timeout = timeout;
    }

    public Waypoint(Pose2d pose, boolean is_continuous) {
        this(pose, is_continuous, true, DEFAULT_INTERMEDIATE_TOLERANCE, false, false, 0.0);
    }

    public Waypoint(Pose2d pose) {
        this(pose, false, true, DEFAULT_INTERMEDIATE_TOLERANCE, false, false, 0.0);
    }

    public Waypoint(Waypoint other)
    {
        this.waypoint_name = other.waypoint_name;
        this.pose = other.pose;
        this.is_continuous = other.is_continuous;
        this.ignore_orientation = other.ignore_orientation;
        this.intermediate_tolerance = other.intermediate_tolerance;
        this.ignore_obstacles = other.ignore_obstacles;
        this.ignore_walls = other.ignore_walls;
        this.timeout = other.timeout;
    }

    public Waypoint makeWithTolerance(double intermediate_tolerance) {
        Waypoint other = new Waypoint(this);
        other.intermediate_tolerance = intermediate_tolerance;
        return other;
    }

    public Waypoint makeContinuous(boolean is_continuous) {
        Waypoint other = new Waypoint(this);
        other.is_continuous = is_continuous;
        return other;
    }

    public Waypoint makeIgnoreOrientation(boolean ignore_orientation) {
        Waypoint other = new Waypoint(this);
        other.ignore_orientation = ignore_orientation;
        return other;
    }

    public Waypoint makeIgnoreObstacles(boolean ignore_obstacles) {
        Waypoint other = new Waypoint(this);
        other.ignore_obstacles = ignore_obstacles;
        return other;
    }

    public Waypoint makeIgnoreWalls(boolean ignore_walls) {
        Waypoint other = new Waypoint(this);
        other.ignore_walls = ignore_walls;
        return other;
    }

    public Waypoint makeWithTimeout(double timeout) {
        Waypoint other = new Waypoint(this);
        other.timeout = timeout;
        return other;
    }
}
