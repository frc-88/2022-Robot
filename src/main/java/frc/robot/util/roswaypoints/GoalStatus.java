package frc.robot.util.roswaypoints;

public enum GoalStatus {
    IDLE(0),
    RUNNING(1),
    FINISHED(2),
    FAILED(3),
    INVALID(-1);
    
    public int status;

    private GoalStatus(int status) {
        this.status = status;
    }

    public static GoalStatus getStatus(int status) {
        for (GoalStatus item : GoalStatus.values()) {
            if (status == item.status) {
                return item;
            }
        }
        return INVALID;
    }
}
