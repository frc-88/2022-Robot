// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.coprocessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.coprocessor.roswaypoints.GoalStatus;
import frc.robot.util.coprocessor.roswaypoints.Waypoint;

public class CoprocessorBase {
    protected final long DEFAULT_MESSAGE_TIMEOUT = 1_000_000;

    protected final ChassisInterface chassis;

    protected VelocityCommand command = new VelocityCommand();
    protected MessageTimer commandTimer = new MessageTimer(DEFAULT_MESSAGE_TIMEOUT);
    
    protected Pose2d globalPose = new Pose2d();
    protected MessageTimer globalPoseTimer = new MessageTimer(DEFAULT_MESSAGE_TIMEOUT);

    protected MessageTimer goalStatusTimer = new MessageTimer(DEFAULT_MESSAGE_TIMEOUT);
    protected GoalStatus goalStatus = GoalStatus.INVALID;

    protected int numSentGoals = 0;

    protected ArrayList<Double> jointCommandValues = new ArrayList<>();
    protected ArrayList<MessageTimer> jointCommandTimers = new ArrayList<>();

    protected Map<String, Pose2d> waypoints = new HashMap<>();

    protected Map<String, Map<Integer, GameObject>> gameObjects = new HashMap<>();

    protected LaserScanObstacleTracker laserObstacles = new LaserScanObstacleTracker();

    protected ZoneManager zoneManager = new ZoneManager();

    public CoprocessorBase(ChassisInterface chassis) {
        this.chassis = chassis;
        laserObstacles.setBoundingBox(chassis.getBoundingBox());
    }

    public void update() {
        
    }

    public void stopComms() {
        
    }

    public void startComms() {
        
    }

    public boolean isConnected() {
        return true;
    }

    /***
     * Getters for data received from coprocessor
     */

    public boolean isCommandActive() {
        return commandTimer.isActive();
    }

    public VelocityCommand getCommand() {
        return command;
    }

    public Pose2d getGlobalPose() {
        return globalPose;
    }

    public boolean isGlobalPoseActive() {
        return globalPoseTimer.isActive();
    }

    public GoalStatus getGoalStatus() {
        return goalStatus;
    }

    public boolean isGoalStatusActive() {
        return goalStatusTimer.isActive();
    }

    public double getJointCommand(int jointIndex) {
        return jointCommandValues.get(jointIndex);
    }

    public boolean isJointCommandActive(int jointIndex) {
        return jointCommandTimers.get(jointIndex).isActive();
    }

    public Pose2d getWaypoint(String waypointName) {
        return waypoints.get(waypointName);
    }

    public void putWaypoint(String waypointName, Pose2d pose) {
        waypoints.put(waypointName, pose);
    }

    public boolean doesWaypointExist(String waypointName) {
        return waypoints.containsKey(waypointName);
    }

    public Set<String> getWaypointNames() {
        return waypoints.keySet();
    }

    /***
     * Setters for sending data to the coprocessor
     */
    
    public void sendGoal(Waypoint waypoint) {
        // String waypointName = WaypointMap.parseWaypointName(waypoint.waypoint_name);
        numSentGoals++;
    }

    public void executeGoal() {
        numSentGoals = 0;
    }

    public void cancelGoal() {
        
    }
    
    public void resetPlan() {
        numSentGoals = 0;
    }

    public void sendMatchStatus(boolean is_autonomous, double match_timer, DriverStation.Alliance team_color) {
        
    }

    public void setPoseEstimate(Pose2d poseEstimation) {
        
    }

    public void setJointPosition(int index, double position) {
        
    }

    public String parseObjectName(String objectName) {
        return Helpers.parseName(objectName);
    }

    public GameObject getNearestGameObject(String objectName)
    {
        objectName = parseObjectName(objectName);
        double min_dist = -1.0;
        String min_obj_name = "";
        int min_obj_index = -1;
        for (String name : gameObjects.keySet()) {
            for (Integer index : gameObjects.get(name).keySet()) {
                GameObject gameObject = gameObjects.get(name).get(index);
                if (gameObject.getName().equals(objectName)) {
                    double distance = gameObject.getDistance();
                    if (min_dist < 0.0 || distance < min_dist) {
                        min_dist = distance;
                        min_obj_name = name;
                        min_obj_index = index;
                    }
                }
            }
        }
        if (min_obj_name.length() == 0 || min_obj_index < 0) {
            System.out.println(objectName + " doesn't exist in object table!");
            return new GameObject("", 0);
        }
        return gameObjects.get(min_obj_name).get(min_obj_index);
    }

    public Set<GameObject> getGameObjects(String objectName) {
        Set<GameObject> objects = new HashSet<>();
        for (String name : gameObjects.keySet()) {
            for (Integer index : gameObjects.get(name).keySet()) {
                GameObject gameObject = gameObjects.get(name).get(index);
                if (gameObject.getName().equals(objectName)) {
                    objects.add(gameObject);
                }
            }
        }
        return objects;
    }

    public GameObject getFirstGameObject(String objectName) {
        for (String name : gameObjects.keySet()) {
            for (Integer index : gameObjects.get(name).keySet()) {
                GameObject gameObject = gameObjects.get(name).get(index);
                if (gameObject.getName().equals(objectName)) {
                    return gameObject;
                }
            }
        }
        return new GameObject("", 0);
    }

    public LaserScanObstacleTracker getLaserScanObstacles() {
        return laserObstacles;
    }

    public boolean areZonesValid() {
        return zoneManager.isValid();
    }

    public ZoneInfo getNearestNoGoZone() {
        return zoneManager.getNearestNoGoZone();
    }

    public ZoneInfo getNearestZone() {
        return zoneManager.getNearestZone();
    }

    public ZoneManager getZoneManager() {
        return zoneManager;
    }

    public void setNoGoZones(String[] names) {
        zoneManager.setNoGoes(names);
    }

    public void setNoGoZone(String name) {
        zoneManager.setNoGo(name);
    }

    public void removeNoGoZone(String name) {
        zoneManager.removeNoGo(name);
    }

    /**
     * @param heading direction of travel of the robot from the robot's perspective
     * @param reverseFanRadians range of accepted angles for each extended beam
     * @param distanceRangeMeters if no go zone is further than this distance, ignore it
     * @return
     */
    public boolean isDirectionTowardNoGoZonesAllowed(double heading, double reverseFanRadians, double distanceRangeMeters) {
        Set<String> names = zoneManager.getNoGoNames();
        List<String> in_bound_names = new ArrayList<>(names);
        in_bound_names.removeIf(name -> zoneManager.getZone(name).getDistance() > distanceRangeMeters);
        double angles[] = new double[in_bound_names.size()];

        int index = 0;
        Pose2d robot_pose = getGlobalPose();
        for (String name : in_bound_names) {
            ZoneInfo zone = zoneManager.getZone(name);
            Pose2d nearest_pose = new Pose2d(zone.getNearestX(), zone.getNearestY(), new Rotation2d());
            Pose2d relative_nearest = nearest_pose.relativeTo(robot_pose);
            angles[index++] = Math.atan2(
                relative_nearest.getY(),
                relative_nearest.getX()
            );
        }
        return Helpers.isDirectionAllowed(heading, angles, reverseFanRadians);
    }
}
