package frc.robot.util.coprocessor;

import edu.wpi.first.wpilibj.RobotController;

public class MessageTimer {
    private long last_time = 0;
    private long active_time_threshold_us;

    public MessageTimer(long active_time_threshold_us) {
        this.active_time_threshold_us = active_time_threshold_us;
    }

    public void reset() {
        last_time = getTime();
    }

    public boolean isActive() {
        return getTime() - last_time < active_time_threshold_us;
    }

    public long getLastActive() {
        return last_time;
    }

    private long getTime() {
        return RobotController.getFPGATime();
    }
}
