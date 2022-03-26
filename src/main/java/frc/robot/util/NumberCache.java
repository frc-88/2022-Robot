package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

public class NumberCache {
    
    private static Map<String, Double> m_map = new HashMap<>();

    public static boolean hasValue(String key) {
        return m_map.containsKey(key);
    }

    public static double getValue(String key) {
        return m_map.get(key);
    }

    public static double pushValue(String key, double value) {
        m_map.put(key, value);
        return value;
    }

    public static void clear() {
        m_map.clear();
    }

}
