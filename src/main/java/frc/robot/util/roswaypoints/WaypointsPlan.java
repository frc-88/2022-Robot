package frc.robot.util.roswaypoints;

import java.util.ArrayList;

import frc.robot.util.coprocessortable.CoprocessorTable;

public class WaypointsPlan {
    private ArrayList<Waypoint> waypoints = new ArrayList<>();
    private final CoprocessorTable coprocessor;

    public WaypointsPlan(CoprocessorTable coprocessor) {
        this.coprocessor = coprocessor;
    }
    
    public void addWaypoint(Waypoint waypoint) {
        waypoints.add(waypoint);
    }

    public void sendWaypoints() {
        resetPlan();
        
        System.out.println("Sending waypoints plan of length " + this.waypoints.size());
        for (int index = 0; index < this.waypoints.size(); index++) {
            coprocessor.sendGoal(this.waypoints.get(index));
        }
        coprocessor.executeGoal();
        System.out.println("Executing plan");
    }

    public void cancelPlan() {
        coprocessor.cancelGoal();
    }

    public void resetPlan() {
        coprocessor.resetPlan();
    }
}
