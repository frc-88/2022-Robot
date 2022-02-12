package frc.robot.util.roswaypoints;

import java.util.ArrayList;

import frc.robot.util.tunnel.ROSInterface;

public class WaypointsPlan {
    private ArrayList<Waypoint> waypoints = new ArrayList<>();
    private final ROSInterface tunnel_interface;

    public WaypointsPlan(ROSInterface tunnel_interface) {
        this.tunnel_interface = tunnel_interface;
    }
    
    public void addWaypoint(Waypoint waypoint) {
        waypoints.add(waypoint);
    }

    public void sendWaypoints() {
        resetPlan();
        
        System.out.println("Sending waypoints plan of length " + this.waypoints.size());
        for (int index = 0; index < this.waypoints.size(); index++) {
            tunnel_interface.sendGoal(this.waypoints.get(index));
        }
        tunnel_interface.executeGoal();
        System.out.println("Executing plan");
    }

    public void cancelPlan() {
        tunnel_interface.cancelGoal();
    }

    public void resetPlan() {
        tunnel_interface.resetPlan();
    }
}
