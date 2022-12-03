package frc.robot.util.coprocessor;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class ZoneManager {
    private boolean _is_valid = false;
    private Map<String, ZoneInfo> zones;
    private Set<String> nogo_zones;

    public ZoneManager() {
        _is_valid = false;
        zones = new HashMap<>();
        nogo_zones = new HashSet<>();
    }

    public void setZone(ZoneInfo info) {
        String name = Helpers.parseName(info.getName());
        if (info.isValid()) {
            ZoneInfo parsed_info = new ZoneInfo(name, info.getNearestX(), info.getNearestY(), info.getDistance(), info.getIsInside(), info.getIsNogo());
            zones.put(name, parsed_info);
        }
        else {
            System.out.println("Skip adding invalid zone: " + name);
        }
    }

    public void setZone(String name, double nearest_x, double nearest_y, double distance, boolean is_inside, boolean is_nogo) {
        setZone(new ZoneInfo(name, nearest_x, nearest_y, distance, is_inside, is_nogo));
    }

    public void setNoGoes(String[] names) {
        nogo_zones = new HashSet<>();
        for (String name : names) {
            setNoGo(name);
        }
    }

    public void setNoGo(String name) {
        nogo_zones.add(Helpers.parseName(name));
    }

    public void removeNoGo(String name) {
        nogo_zones.remove(Helpers.parseName(name));
    }

    public boolean isNoGo(String name) {
        return nogo_zones.contains(Helpers.parseName(name));
    }

    public Set<String> getNames() {
        return zones.keySet();
    }

    public Set<String> getNoGoNames() {
        return nogo_zones;
    }

    public void setValid(boolean is_valid) {
        _is_valid = is_valid;
    }

    public boolean isValid() {
        return _is_valid;
    }

    public ZoneInfo getNearestNoGoZone() {
        return getNearestZone(true);
    }

    public ZoneInfo getNearestZone() {
        return getNearestZone(false);
    }
    public ZoneInfo getNearestZone(boolean useNoGo) {
        double min_distance = 0.0;
        String min_zone = "";
        for (String name : zones.keySet()) {
            ZoneInfo zone = zones.get(name);
            if (useNoGo && !zone.getIsNogo()) {
                continue;
            }
            if (min_zone.length() == 0 || zone.getDistance() < min_distance) {
                min_distance = zone.getDistance();
                min_zone = name;
            }
        }
        if (min_zone.length() == 0) {
            return new ZoneInfo(false);
        }
        else {
            return zones.get(min_zone);
        }
    }

    public ZoneInfo getZone(String name) {
        return zones.get(name);
    }
}
