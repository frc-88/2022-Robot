package frc.robot.util.coprocessortable;

import java.util.ArrayList;
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
import frc.robot.util.roswaypoints.GoalStatus;
import frc.robot.util.roswaypoints.Waypoint;
import frc.robot.util.roswaypoints.WaypointMap;

public class CoprocessorTable {
    protected ChassisInterface chassis;

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

    private NetworkTable imuTable;
    private NetworkTableEntry imuEntryAccelX;
    private NetworkTableEntry imuEntryAccelY;
    private NetworkTableEntry imuEntryAccelZ;
    private NetworkTableEntry imuEntryGyroX;
    private NetworkTableEntry imuEntryGyroY;
    private NetworkTableEntry imuEntryGyroZ;
    private NetworkTableEntry imuEntryAngleX;
    private NetworkTableEntry imuEntryAngleY;
    private NetworkTableEntry imuEntryAngleZ;
    private NetworkTableEntry imuEntryUpdate;

    private NetworkTable cmdVelTable;
    private NetworkTableEntry cmdVelEntryX;
    private NetworkTableEntry cmdVelEntryY;
    private NetworkTableEntry cmdVelEntryT;
    private NetworkTableEntry cmdVelEntryUpdate;
    private VelocityCommand command = new VelocityCommand();
    private MessageTimer commandTimer = new MessageTimer(1_000_000);

    private NetworkTable globalPoseTable;
    private NetworkTableEntry globalPoseEntryX;
    private NetworkTableEntry globalPoseEntryY;
    private NetworkTableEntry globalPoseEntryT;
    private NetworkTableEntry globalPoseEntryUpdate;
    private Pose2d globalPose = new Pose2d();
    private MessageTimer globalPoseTimer = new MessageTimer(1_000_000);

    private NetworkTable goalStatusTable;
    private NetworkTableEntry goalStatusEntry;
    private NetworkTableEntry goalStatusUpdateEntry;
    private MessageTimer goalStatusTimer = new MessageTimer(1_000_000);
    protected GoalStatus goalStatus = GoalStatus.INVALID;

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
    private NetworkTableEntry waypointInterruptableByEntry;
    private NetworkTableEntry waypointTimeoutEntry;
    private int numSentGoals = 0;

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
    ArrayList<Double> jointCommandValues = new ArrayList<>();
    ArrayList<MessageTimer> jointCommandTimers = new ArrayList<>();

    private NetworkTable waypointsTable;

    public CoprocessorTable(ChassisInterface chassis, String address, int port, double updateInterval) {
        this.chassis = chassis;
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

        imuTable = rootTable.getSubTable("imu");
        imuEntryAccelX = imuTable.getEntry("accel/x");
        imuEntryAccelY = imuTable.getEntry("accel/y");
        imuEntryAccelZ = imuTable.getEntry("accel/z");
        imuEntryGyroX = imuTable.getEntry("gyro/x");
        imuEntryGyroY = imuTable.getEntry("gyro/y");
        imuEntryGyroZ = imuTable.getEntry("gyro/z");
        imuEntryAngleX = imuTable.getEntry("angle/x");
        imuEntryAngleY = imuTable.getEntry("angle/y");
        imuEntryAngleZ = imuTable.getEntry("angle/z");
        imuEntryUpdate = imuTable.getEntry("update");

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

        sendOdometry();
        sendImu(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
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
        
        sendOdometry();
        
        sendMatchStatus(
            DriverStation.isAutonomous(),
            DriverStation.getMatchTime(),
            DriverStation.getAlliance()
        );
    }

    public void sendOdometry()
    {
        Pose2d pose = this.chassis.getOdometryPose();
        ChassisSpeeds velocity = this.chassis.getChassisVelocity();
        odomEntryX.setDouble(pose.getX());
        odomEntryY.setDouble(pose.getY());
        odomEntryT.setDouble(pose.getRotation().getRadians());
        odomEntryVx.setDouble(velocity.vxMetersPerSecond);
        odomEntryVy.setDouble(velocity.vyMetersPerSecond);
        odomEntryVt.setDouble(velocity.omegaRadiansPerSecond);
        odomEntryUpdate.setDouble(getTime());
    }

    public void sendImu(double ax, double ay, double az, double gx, double gy, double gz, double tx, double ty, double tz)
    {
        imuEntryAccelX.setDouble(ax);
        imuEntryAccelY.setDouble(ay);
        imuEntryAccelZ.setDouble(az);
        imuEntryGyroX.setDouble(gx);
        imuEntryGyroY.setDouble(gy);
        imuEntryGyroZ.setDouble(gz);
        imuEntryAngleX.setDouble(tx);
        imuEntryAngleY.setDouble(ty);
        imuEntryAngleZ.setDouble(tz);
        imuEntryUpdate.setDouble(getTime());
    }

    public void sendImu(double ax, double ay, double gz, double tz)
    {
        // Only send 2D parameters
        imuEntryAccelX.setDouble(ax);
        imuEntryAccelY.setDouble(ay);
        imuEntryGyroZ.setDouble(gz);
        imuEntryAngleZ.setDouble(tz);
        imuEntryUpdate.setDouble(getTime());
    }

    public NetworkTable getWaypointsTable() {
        return waypointsTable;
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

    public double getUpdateInterval() {
        return updateInterval;
    }

    public double getJointCommand(int jointIndex) {
        return jointCommandValues.get(jointIndex);
    }

    public boolean isJointCommandActive(int jointIndex) {
        return jointCommandTimers.get(jointIndex).isActive();
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
        waypointInterruptableByEntry = waypointSegmentTable.getEntry("interruptable_by");
        waypointTimeoutEntry = waypointSegmentTable.getEntry("timeout");

        waypointIsContinuousEntry.setValue(waypoint.is_continuous);
        waypointIgnoreOrientationEntry.setValue(waypoint.ignore_orientation);
        waypointIntermediateToleranceEntry.setValue(waypoint.intermediate_tolerance);
        waypointIgnoreObstaclesEntry.setValue(waypoint.ignore_obstacles);
        waypointIgnoreWallsEntry.setValue(waypoint.ignore_walls);
        waypointInterruptableByEntry.setValue(waypoint.interruptableBy);
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
        String team_name = "";
        if (team_color == Alliance.Red) {
            team_name = "red";
        }
        else if (team_color == Alliance.Blue) {
            team_name = "blue";
        }
        
        isAutonomousEntry.setBoolean(is_autonomous);
        teamColorEntry.setString(team_name);
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
            jointCommandTimers.set(index, new MessageTimer(1_000_000));
        }

    }
}
