package frc.robot.util.coprocessor.networktables;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.coprocessor.ChassisInterface;
import frc.robot.util.coprocessor.GameObject;
import frc.robot.util.coprocessor.LaserScanObstacleTracker;
import frc.robot.util.coprocessor.MessageTimer;
import frc.robot.util.coprocessor.CoprocessorBase;
import frc.robot.util.roswaypoints.GoalStatus;
import frc.robot.util.roswaypoints.Waypoint;
import frc.robot.util.roswaypoints.WaypointMap;

public class CoprocessorTable extends CoprocessorBase {
    private NetworkTableInstance instance;
    private String address;
    private int port;
    protected NetworkTable rootTable;
    private double updateInterval = 0.01;
    private NetworkTableEntry pingEntry;
    private NetworkTableEntry pingReturnEntry;

    private NetworkTable odomTable;
    private NetworkTableEntry odomEntryX;
    private NetworkTableEntry odomEntryY;
    private NetworkTableEntry odomEntryT;
    private NetworkTableEntry odomEntryVx;
    private NetworkTableEntry odomEntryVy;
    private NetworkTableEntry odomEntryVt;
    private NetworkTableEntry odomEntryUpdate;

    private NetworkTable cmdVelTable;
    private NetworkTableEntry cmdVelEntryX;
    private NetworkTableEntry cmdVelEntryY;
    private NetworkTableEntry cmdVelEntryT;
    private NetworkTableEntry cmdVelEntryUpdate;

    private NetworkTable globalPoseTable;
    private NetworkTableEntry globalPoseEntryX;
    private NetworkTableEntry globalPoseEntryY;
    private NetworkTableEntry globalPoseEntryT;
    private NetworkTableEntry globalPoseEntryUpdate;

    private NetworkTable goalStatusTable;
    private NetworkTableEntry goalStatusEntry;
    private NetworkTableEntry goalStatusUpdateEntry;

    private NetworkTable odomResetTable;
    private NetworkTableEntry odomResetEntryX;
    private NetworkTableEntry odomResetEntryY;
    private NetworkTableEntry odomResetEntryT;
    private NetworkTableEntry odomResetEntryUpdate;

    private NetworkTable matchTable;
    private NetworkTableEntry matchTimerEntry;
    private NetworkTableEntry isAutonomousEntry;
    private NetworkTableEntry teamColorEntry;
    private NetworkTableEntry matchUpdateEntry;

    private NetworkTable waypointPlanTable;
    private NetworkTableEntry waypointNameEntry;
    private NetworkTableEntry waypointPoseXEntry;
    private NetworkTableEntry waypointPoseYEntry;
    private NetworkTableEntry waypointPoseTEntry;
    private NetworkTableEntry waypointIsContinuousEntry;
    private NetworkTableEntry waypointIgnoreOrientationEntry;
    private NetworkTableEntry waypointIntermediateToleranceEntry;
    private NetworkTableEntry waypointIgnoreObstaclesEntry;
    private NetworkTableEntry waypointIgnoreWallsEntry;
    private NetworkTableEntry waypointTimeoutEntry;

    private NetworkTable planControlTable;
    private NetworkTableEntry execPlanEntry;
    private NetworkTableEntry execUpdatePlanEntry;
    private NetworkTableEntry resetPlanEntry;
    private NetworkTableEntry cancelPlanEntry;

    private NetworkTable poseEstTable;
    private NetworkTableEntry poseEstEntryX;
    private NetworkTableEntry poseEstEntryY;
    private NetworkTableEntry poseEstEntryT;
    private NetworkTableEntry poseEstEntryUpdate;

    private NetworkTable jointsTable;
    private NetworkTable jointCommandsTable;
    ArrayList<NetworkTableEntry> jointCommandEntries = new ArrayList<>();

    private NetworkTable waypointsTable;
    protected Map<String, NetworkTableEntry> waypointXEntries = new HashMap<>();
    protected Map<String, NetworkTableEntry> waypointYEntries = new HashMap<>();
    protected Map<String, NetworkTableEntry> waypointTEntries = new HashMap<>();

    private NetworkTable objectTable;

    private NetworkTable laserScanTable;
    private NetworkTableEntry laserScanEntryXs;
    private NetworkTableEntry laserScanEntryYs;
    private double[] laserXs = new double[0];
    private double[] laserYs = new double[0];

    public CoprocessorTable(ChassisInterface chassis, String address, int port, double updateInterval) {
        super(chassis);
        this.address = address;
        this.port = port;

        instance = NetworkTableInstance.create();
        instance.startClient(address, port);
        instance.setUpdateRate(updateInterval);
        this.updateInterval = updateInterval;

        rootTable = instance.getTable("ROS");
        
        pingEntry = rootTable.getEntry("ping");
        pingReturnEntry = rootTable.getEntry("ping_return");

        pingEntry.addListener((notification) -> {
            pingReturnEntry.setDouble(notification.getEntry().getDouble(0.0));
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        odomTable = rootTable.getSubTable("odom");
        odomEntryX = odomTable.getEntry("x");
        odomEntryY = odomTable.getEntry("y");
        odomEntryT = odomTable.getEntry("t");
        odomEntryVx = odomTable.getEntry("vx");
        odomEntryVy = odomTable.getEntry("vy");
        odomEntryVt = odomTable.getEntry("vt");
        odomEntryUpdate = odomTable.getEntry("update");

        cmdVelTable = rootTable.getSubTable("cmd_vel");
        cmdVelEntryX = cmdVelTable.getEntry("x");
        cmdVelEntryY = cmdVelTable.getEntry("y");
        cmdVelEntryT = cmdVelTable.getEntry("t");
        cmdVelEntryUpdate = cmdVelTable.getEntry("update");
        cmdVelEntryUpdate.addListener(this::cmdVelCallback, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        globalPoseTable = rootTable.getSubTable("global");
        globalPoseEntryX = globalPoseTable.getEntry("x");
        globalPoseEntryY = globalPoseTable.getEntry("y");
        globalPoseEntryT = globalPoseTable.getEntry("t");
        globalPoseEntryUpdate = globalPoseTable.getEntry("update");
        globalPoseEntryUpdate.addListener(this::globalPoseCallback, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        goalStatusTable = rootTable.getSubTable("goal_status");
        goalStatusEntry = goalStatusTable.getEntry("status");
        goalStatusUpdateEntry = goalStatusTable.getEntry("update");
        goalStatusUpdateEntry.addListener(this::goalStatusCallback, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        odomResetTable = rootTable.getSubTable("reset_odom");
        odomResetEntryX = odomResetTable.getEntry("x");
        odomResetEntryY = odomResetTable.getEntry("y");
        odomResetEntryT = odomResetTable.getEntry("t");
        odomResetEntryUpdate = odomResetTable.getEntry("update");
        odomResetEntryUpdate.addListener(this::resetOdomCallback, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        matchTable = rootTable.getSubTable("match");
        matchTimerEntry = matchTable.getEntry("time");
        isAutonomousEntry = matchTable.getEntry("is_auto");
        teamColorEntry = matchTable.getEntry("team_color");
        matchUpdateEntry = matchTable.getEntry("update");

        waypointPlanTable = rootTable.getSubTable("goal");

        planControlTable = rootTable.getSubTable("plan");
        execPlanEntry = planControlTable.getEntry("exec");
        execUpdatePlanEntry = planControlTable.getEntry("exec_update");
        resetPlanEntry = planControlTable.getEntry("reset");
        cancelPlanEntry = planControlTable.getEntry("cancel");
        
        poseEstTable = rootTable.getSubTable("pose_est");
        poseEstEntryX = poseEstTable.getEntry("x");
        poseEstEntryY = poseEstTable.getEntry("y");
        poseEstEntryT = poseEstTable.getEntry("t");
        poseEstEntryUpdate = poseEstTable.getEntry("update");

        jointsTable = rootTable.getSubTable("joints");
        jointCommandsTable = jointsTable.getSubTable("commands");

        waypointsTable = rootTable.getSubTable("waypoints");
        waypointsTable.addSubTableListener((parent, name, table) -> {newWaypointCallback(name);}, true);

        objectTable = rootTable.getSubTable("detections");
        objectTable.addSubTableListener((parent, name, table) -> {newObjectCallback(name);}, true);

        laserScanTable = rootTable.getSubTable("laser");
        laserScanEntryXs = laserScanTable.getEntry("xs");
        laserScanEntryYs = laserScanTable.getEntry("ys");
        laserScanEntryXs.addListener(this::scanCallback, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    private void cmdVelCallback(EntryNotification notification) {
        command.vx = cmdVelEntryX.getDouble(0.0);
        command.vy = cmdVelEntryY.getDouble(0.0);
        command.vt = cmdVelEntryT.getDouble(0.0);
        commandTimer.reset();
    }

    private void globalPoseCallback(EntryNotification notification) {
        double x = globalPoseEntryX.getDouble(0.0);
        double y = globalPoseEntryY.getDouble(0.0);
        double theta = globalPoseEntryT.getDouble(0.0);
        globalPose = new Pose2d(x, y, new Rotation2d(theta));
        globalPoseTimer.reset();
    }

    private void goalStatusCallback(EntryNotification notification) {
        goalStatus = GoalStatus.getStatus((int)goalStatusEntry.getDouble(-1.0));
        goalStatusTimer.reset();
    }

    private void resetOdomCallback(EntryNotification notification) {
        double x = odomResetEntryX.getDouble(0.0);
        double y = odomResetEntryY.getDouble(0.0);
        double theta = odomResetEntryT.getDouble(0.0);
        this.chassis.resetPosition(new Pose2d(x, y, new Rotation2d(theta)));
    }

    private void jointCommandCallback(EntryNotification notification, int jointIndex, NetworkTableEntry valueEntry) {
        if (Objects.isNull(jointCommandValues.get(jointIndex))) {
            System.out.println("WARNING! jointCommandValues is null! Can't set joint command");
            return;
        }
        if (Objects.isNull(jointCommandTimers.get(jointIndex))) {
            System.out.println("WARNING! jointCommandTimers is null! Can't set joint command");
            return;
        }
        jointCommandValues.set(jointIndex, valueEntry.getDouble(0.0));
        jointCommandTimers.get(jointIndex).reset();
    }

    public NetworkTableInstance getNetworkTableInstance() {
        return instance;
    }

    public NetworkTable getRootTable() {
        return rootTable;
    }

    public void update() {
        if (!instance.isConnected()) {
            return;
        }
        
        Pose2d pose = this.chassis.getOdometryPose();
        ChassisSpeeds velocity = this.chassis.getChassisSpeeds();
        odomEntryX.setDouble(pose.getX());
        odomEntryY.setDouble(pose.getY());
        odomEntryT.setDouble(pose.getRotation().getRadians());
        odomEntryVx.setDouble(velocity.vxMetersPerSecond);
        odomEntryVy.setDouble(velocity.vyMetersPerSecond);
        odomEntryVt.setDouble(velocity.omegaRadiansPerSecond);
        odomEntryUpdate.setDouble(getTime());

        sendMatchStatus(
            DriverStation.isAutonomous(),
            DriverStation.getMatchTime(),
            DriverStation.getAlliance()
        );
    }

    public NetworkTable getWaypointsTable() {
        return waypointsTable;
    }

    private void newWaypointCallback(String name) {
        NetworkTableEntry xEntry = waypointsTable.getSubTable(name).getEntry("x");
        NetworkTableEntry yEntry = waypointsTable.getSubTable(name).getEntry("y");
        NetworkTableEntry tEntry = waypointsTable.getSubTable(name).getEntry("theta");
        waypointXEntries.put(name, xEntry);
        waypointYEntries.put(name, yEntry);
        waypointTEntries.put(name, tEntry);
        Pose2d pose = new Pose2d(
            waypointXEntries.get(name).getDouble(0.0),
            waypointYEntries.get(name).getDouble(0.0),
            new Rotation2d(waypointTEntries.get(name).getDouble(0.0))
        );
        waypoints.put(name, pose);
        xEntry.addListener((notification) -> this.waypointXEntryCallback(name), EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        yEntry.addListener((notification) -> this.waypointYEntryCallback(name), EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        tEntry.addListener((notification) -> this.waypointTEntryCallback(name), EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    private void waypointXEntryCallback(String waypointName) {
        Pose2d old_pose = waypoints.get(waypointName);
        System.out.println("waypointXEntryCallback: " + old_pose);
        Pose2d new_pose = new Pose2d(
            waypointXEntries.get(waypointName).getDouble(0.0),
            old_pose.getY(),
            old_pose.getRotation()
        );
        putWaypoint(waypointName, new_pose);
    }
    private void waypointYEntryCallback(String waypointName) {
        Pose2d old_pose = waypoints.get(waypointName);
        Pose2d new_pose = new Pose2d(
            old_pose.getX(),
            waypointYEntries.get(waypointName).getDouble(0.0),
            old_pose.getRotation()
        );
        putWaypoint(waypointName, new_pose);
    }
    private void waypointTEntryCallback(String waypointName) {
        Pose2d old_pose = waypoints.get(waypointName);
        Pose2d new_pose = new Pose2d(
            old_pose.getX(),
            old_pose.getY(),
            new Rotation2d(waypointTEntries.get(waypointName).getDouble(0.0))
        );
        putWaypoint(waypointName, new_pose);
    }

    public void stopComms() {
        if (instance.isConnected()) {
            instance.stopClient();
        }
    }

    public void startComms() {
        if (!instance.isConnected()) {
            instance.startClient(address, port);
        }
    }

    public boolean isConnected() {
        return instance.isConnected();
    }

    protected double getTime() {
        return RobotController.getFPGATime() * 1E-6;
    }

    public double getUpdateInterval() {
        return updateInterval;
    }

    /***
     * Setters for sending data to the coprocessor
     */
    
    public void sendGoal(Waypoint waypoint) {
        String waypointName = WaypointMap.parseWaypointName(waypoint.waypoint_name);
        setCommonWaypointEntries(numSentGoals, waypoint);
        waypointNameEntry.setValue(waypointName);  // if the name is not empty, pose entries are ignored by planner
        waypointPoseXEntry.setValue(waypoint.pose.getX());
        waypointPoseYEntry.setValue(waypoint.pose.getY());
        waypointPoseTEntry.setValue(waypoint.pose.getRotation().getRadians());
        numSentGoals++;
    }

    private void setCommonWaypointEntries(int index, Waypoint waypoint) {
        NetworkTable waypointSegmentTable = waypointPlanTable.getSubTable(String.valueOf(index));
        waypointNameEntry = waypointSegmentTable.getEntry("name");
        waypointPoseXEntry = waypointSegmentTable.getEntry("x");
        waypointPoseYEntry = waypointSegmentTable.getEntry("y");
        waypointPoseTEntry = waypointSegmentTable.getEntry("t");
        waypointIsContinuousEntry = waypointSegmentTable.getEntry("is_continuous");
        waypointIgnoreOrientationEntry = waypointSegmentTable.getEntry("ignore_orientation");
        waypointIntermediateToleranceEntry = waypointSegmentTable.getEntry("intermediate_tolerance");
        waypointIgnoreObstaclesEntry = waypointSegmentTable.getEntry("ignore_obstacles");
        waypointIgnoreWallsEntry = waypointSegmentTable.getEntry("ignore_walls");
        waypointTimeoutEntry = waypointSegmentTable.getEntry("timeout");

        waypointIsContinuousEntry.setValue(waypoint.is_continuous);
        waypointIgnoreOrientationEntry.setValue(waypoint.ignore_orientation);
        waypointIntermediateToleranceEntry.setValue(waypoint.intermediate_tolerance);
        waypointIgnoreObstaclesEntry.setValue(waypoint.ignore_obstacles);
        waypointIgnoreWallsEntry.setValue(waypoint.ignore_walls);
        waypointTimeoutEntry.setValue(waypoint.timeout);
    }

    public void executeGoal() {
        execPlanEntry.setValue(numSentGoals);
        execUpdatePlanEntry.setDouble(getTime());
        numSentGoals = 0;
    }

    public void cancelGoal() {
        cancelPlanEntry.setDouble(getTime());
    }
    
    public void resetPlan() {
        resetPlanEntry.setDouble(getTime());
        numSentGoals = 0;
    }

    public void sendMatchStatus(boolean is_autonomous, double match_timer, DriverStation.Alliance team_color) {
        isAutonomousEntry.setBoolean(is_autonomous);
        teamColorEntry.setString(getTeamName(team_color));
        matchTimerEntry.setDouble(match_timer);
        matchUpdateEntry.setDouble(getTime());
    }

    public void setPoseEstimate(Pose2d poseEstimation) {
        poseEstEntryX.setDouble(poseEstimation.getX());
        poseEstEntryY.setDouble(poseEstimation.getY());
        poseEstEntryT.setDouble(poseEstimation.getRotation().getRadians());
        poseEstEntryUpdate.setDouble(getTime());
    }

    public void setJointPosition(int index, double position) {
        jointsTable.getEntry(String.valueOf(index)).setDouble(position);

        while (index >= jointCommandEntries.size()) {
            jointCommandEntries.add(null);
            jointCommandValues.add(null);
            jointCommandTimers.add(null);
        }
        if (Objects.isNull(jointCommandEntries.get(index))) {
            NetworkTableEntry jointUpdateEntry = jointCommandsTable.getEntry("update/" + String.valueOf(index));
            NetworkTableEntry jointValueEntry = jointCommandsTable.getEntry("value/" + String.valueOf(index));
            jointUpdateEntry.addListener(
                (notification) -> jointCommandCallback(notification, index, jointValueEntry), 
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
            jointCommandEntries.set(index, jointUpdateEntry);
            jointCommandValues.set(index, 0.0);
            jointCommandTimers.set(index, new MessageTimer(DEFAULT_MESSAGE_TIMEOUT));
        }
    }

    private void newObjectCallback(String name) {
        gameObjects.put(name, new GameObject(name));
        NetworkTableEntry updateEntry = objectTable.getSubTable(name).getEntry("update");
        updateEntry.addListener((notification) -> this.objectEntryCallback(name, notification), EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        System.out.println("Registering object " + name);
    }

    void objectEntryCallback(String objectName, EntryNotification notification)
    {
        GameObject gameObject = gameObjects.get(objectName);
        gameObject.count = (int)objectTable.getSubTable(objectName).getEntry("count").getDouble(0.0);
        gameObject.set(
            objectTable.getSubTable(objectName).getEntry("x").getDouble(0.0), 
            objectTable.getSubTable(objectName).getEntry("y").getDouble(0.0), 
            objectTable.getSubTable(objectName).getEntry("z").getDouble(0.0)
        );
    }

    public String parseObjectName(String objectName) {
        return objectName.replaceAll("<team>", getTeamName(DriverStation.getAlliance()));
    }

    public GameObject getNearestGameObject(String objectName)
    {
        objectName = parseObjectName(objectName);
        if (!gameObjects.containsKey(objectName)) {
            newObjectCallback(objectName);
        }
        if (!objectTable.containsSubTable(objectName)) {
            System.out.println(objectName + " doesn't exist in object table!");
            return new GameObject("");
        }
        return gameObjects.get(objectName);
    }

    private void scanCallback(EntryNotification notification)
    {
        laserXs = laserScanEntryXs.getDoubleArray(laserXs);
        laserYs = laserScanEntryYs.getDoubleArray(laserYs);

        laserObstacles.setPoints(laserXs, laserYs);
    }
}
