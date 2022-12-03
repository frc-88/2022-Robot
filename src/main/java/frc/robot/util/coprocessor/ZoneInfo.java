package frc.robot.util.coprocessor;

public class ZoneInfo {
    private boolean _is_valid = false;
    private String _name = "";
    private double _nearest_x = 0.0;
    private double _nearest_y = 0.0;
    private double _distance = 0.0;
    private boolean _is_inside = false;
    private boolean _is_nogo = false;

    public ZoneInfo(boolean is_valid) {
        _is_valid = is_valid;
    }

    public ZoneInfo(String name, double nearest_x, double nearest_y, double distance, boolean is_inside, boolean is_nogo) {
        _name = name;
        _nearest_x = nearest_x;
        _nearest_y = nearest_y;
        _distance = distance;
        _is_inside = is_inside;
        _is_nogo = is_nogo;
        _is_valid = true;
    }

    public String getName() {
        return _name;
    }
    public double getNearestX() {
        return _nearest_x;
    }
    public double getNearestY() {
        return _nearest_y;
    }
    public double getDistance() {
        return _distance;
    }
    public boolean getIsInside() {
        return _is_inside;
    }
    public boolean getIsNogo() {
        return _is_nogo;
    }
    public boolean isValid() {
        return _is_valid;
    }
}
