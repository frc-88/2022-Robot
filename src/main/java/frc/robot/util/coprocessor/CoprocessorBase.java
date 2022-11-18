// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.coprocessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.roswaypoints.GoalStatus;
import frc.robot.util.roswaypoints.Waypoint;

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

    protected Map<String, GameObject> gameObjects = new HashMap<>();

    protected LaserScanObstacleTracker laserObstacles = new LaserScanObstacleTracker();

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
        System.out.println("waypoints.keySet(): " + waypoints.keySet());
        return waypoints.get(waypointName);
    }

    public void putWaypoint(String waypointName, Pose2d pose) {
        System.out.println("putting waypoint: " + waypointName + ", " + pose);
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

    protected String getTeamName(DriverStation.Alliance team_color) {
        String team_name = "";
        if (team_color == Alliance.Red) {
            team_name = "red";
        }
        else if (team_color == Alliance.Blue) {
            team_name = "blue";
        }
        return team_name;
    }

    public void setPoseEstimate(Pose2d poseEstimation) {
        
    }

    public void setJointPosition(int index, double position) {
        
    }

    public GameObject getNearestGameObject(String objectName) {
        return gameObjects.get(objectName);
    }

    public LaserScanObstacleTracker getLaserScanObstacles() {
        return laserObstacles;
    }
}
