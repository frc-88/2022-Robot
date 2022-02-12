package frc.robot.util.roswaypoints;

public class Waypoint {
    public String waypoint_name = "";
    public boolean is_continuous = false;
    public boolean ignore_orientation = true;
    public double intermediate_tolerance = 0.2;
    public boolean ignore_obstacles = false;
    public boolean ignore_walls = false;

    public Waypoint(String waypoint_name, boolean is_continuous, boolean ignore_orientation, double intermediate_tolerance, boolean ignore_obstacles, boolean ignore_walls)
    {
        this.waypoint_name = waypoint_name;
        this.is_continuous = is_continuous;
        this.ignore_orientation = ignore_orientation;
        this.intermediate_tolerance = intermediate_tolerance;
        this.ignore_obstacles = ignore_obstacles;
        this.ignore_walls = ignore_walls;
    }

    public Waypoint(String waypoint_name, boolean is_continuous) {
        this(waypoint_name, is_continuous, true, 0.2, false, false);
    }

    public Waypoint(String waypoint_name) {
        this(waypoint_name, false, true, 0.2, false, false);
    }

    public Waypoint(Waypoint other)
    {
        this.waypoint_name = other.waypoint_name;
        this.is_continuous = other.is_continuous;
        this.ignore_orientation = other.ignore_orientation;
        this.intermediate_tolerance = other.intermediate_tolerance;
        this.ignore_obstacles = other.ignore_obstacles;
        this.ignore_walls = other.ignore_walls;
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
}
