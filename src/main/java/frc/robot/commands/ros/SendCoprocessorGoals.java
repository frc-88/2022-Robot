package frc.robot.commands.ros;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.roswaypoints.WaypointsPlan;

public class SendCoprocessorGoals extends InstantCommand {
    private WaypointsPlan plan;
    
    public SendCoprocessorGoals(WaypointsPlan plan) {
        this.plan = plan;
    }

    @Override
    public void initialize() {
        System.out.println("Starting SendCoprocessorPlan command");
        this.plan.sendWaypoints();
        System.out.println("SendCoprocessorPlan command finished");
    }
}
