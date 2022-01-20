package frc.robot.util.tunnel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.util.roswaypoints.GoalStatus;
import frc.robot.util.roswaypoints.Waypoint;


public class ROSInterface implements TunnelInterface {
    protected ChassisInterface chassis;
    
    private MessageTimer commandTimer = new MessageTimer(1_000_000);
    private MessageTimer globalPoseTimer = new MessageTimer(1_000_000);
    private MessageTimer goalStatusTimer = new MessageTimer(1_000_000);

    protected VelocityCommand command = new VelocityCommand();
    
    protected Pose2d globalPose = new Pose2d();
    
    protected GoalStatus goalStatus = GoalStatus.INVALID;

    private int num_sent_goals = 0;

    public ROSInterface(ChassisInterface chassis) {
        this.chassis = chassis;
    }

    @Override
    public void packetCallback(TunnelClient tunnel, PacketResult result) {
        String category = result.getCategory();

        if (category.equals("cmd")) {
            // Velocity commands sent by the coprocessor
            command.vx = result.getDouble();
            command.vy = result.getDouble();
            command.vt = result.getDouble();
            commandTimer.setTunnelClient(tunnel);
            commandTimer.reset();
        }
        else if (category.equals("global")) {
            // Global position as declared by the coprocessor
            globalPose = new Pose2d(
                result.getDouble(),
                result.getDouble(),
                new Rotation2d(result.getDouble())
            );
            globalPoseTimer.setTunnelClient(tunnel);
            globalPoseTimer.reset();
        }
        else if (category.equals("gstatus")) {
            // move_base goal status
            int status = result.getInt();
            goalStatus = GoalStatus.getStatus(status);
            
            goalStatusTimer.setTunnelClient(tunnel);
            goalStatusTimer.reset();
        }
        else if (category.equals("ping")) {
            tunnel.writePacket("ping", result.getDouble());
        }
        else if (category.equals("reset")) {
            double x = result.getDouble();
            double y = result.getDouble();
            double theta = result.getDouble();
            this.chassis.resetPosition(new Pose2d(new Translation2d(x, y), new Rotation2d(theta)));
        }
    }

    @Override
    public void update() {
        Pose2d pose = this.chassis.getOdometryPose();
        ChassisSpeeds velocity = this.chassis.getChassisVelocity();
        TunnelServer.writePacket("odom",
            pose.getX(), pose.getY(), pose.getRotation().getRadians(),
            velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, velocity.omegaRadiansPerSecond
        );
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

    /***
     * Setters for sending data to the coprocessor
     */
    
    public void sendGoal(Waypoint waypoint) {
        TunnelServer.writePacket(
            "goal",
            waypoint.waypoint_name,
            waypoint.is_continuous,
            waypoint.ignore_orientation,
            waypoint.intermediate_tolerance,
            waypoint.ignore_obstacles,
            waypoint.ignore_walls
        );
        num_sent_goals++;
    }

    public void executeGoal() {
        System.out.println("Sending execute command. Num waypoints: " + num_sent_goals);
        TunnelServer.writePacket("exec", num_sent_goals);
        num_sent_goals = 0;
    }

    public void cancelGoal() {
        TunnelServer.writePacket("cancel");
    }
    
    public void resetPlan() {
        num_sent_goals = 0;
        TunnelServer.writePacket("reset");
    }

    public void sendMatchStatus(boolean motor_enabled, boolean is_autonomous, double match_timer) {
        TunnelServer.writePacket("match", motor_enabled, is_autonomous, match_timer);
    }

    public void setPoseEstimate(Pose2d poseEstimation) {
        TunnelServer.writePacket("poseest",
            poseEstimation.getTranslation().getX(),
            poseEstimation.getTranslation().getY(),
            poseEstimation.getRotation().getRadians()
        );
    }
}
