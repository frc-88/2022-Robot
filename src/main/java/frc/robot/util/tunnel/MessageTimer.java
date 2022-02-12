package frc.robot.util.tunnel;

import java.util.Objects;

import edu.wpi.first.wpilibj.RobotController;

public class MessageTimer {
    private long last_time = 0;
    private long active_time_threshold_us;
    private TunnelClient client = null;

    public MessageTimer(long active_time_threshold_us) {
        this.active_time_threshold_us = active_time_threshold_us;
    }

    public void reset() {
        last_time = getTime();
    }

    public boolean isActive() {
        return isTunnelOpen() && getTime() - last_time < active_time_threshold_us;
    }

    public void setTunnelClient(TunnelClient client) {
        this.client = client;
    }

    private long getTime() {
        return RobotController.getFPGATime();
    }

    private boolean isTunnelOpen() {
        if (Objects.isNull(client)) {
            return false;
        }
        return client.isAlive() && client.isOpen();
    }
}
