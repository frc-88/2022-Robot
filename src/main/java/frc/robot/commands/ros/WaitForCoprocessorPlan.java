package frc.robot.commands.ros;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.tunnel.ROSInterface;
import frc.robot.subsystems.Drive;
import frc.robot.util.tunnel.TunnelServer;

public class WaitForCoprocessorPlan extends CommandBase {
    private final Drive drive;
    private final ROSInterface tunnel_interface;
    private boolean is_finished = false;
    
    public WaitForCoprocessorPlan(Drive drive, ROSInterface tunnel_interface)
    {
        this.drive = drive;
        this.tunnel_interface = tunnel_interface;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        is_finished = false;
        System.out.println("Starting WaitForCoprocessorPlan command");
    }

    @Override
    public void execute() {
        if (TunnelServer.anyClientsAlive() && tunnel_interface.isCommandActive()) {
            drive.drive(tunnel_interface.getCommand());
        }
        
        switch (tunnel_interface.getGoalStatus()) {
            case RUNNING:
                break;
            case INVALID: 
                System.out.println("Coprocessor entered into state INVALID. Cancelling goal.");
                cancelGoal();
                break;
            case IDLE:
                System.out.println("Coprocessor entered into state IDLE. Cancelling goal.");
                cancelGoal();
                break;

            case FAILED:
                System.out.println("Coprocessor entered into state FAILED. Exiting command.");
                setFinished();
                break;
            case FINISHED:
                setFinished();
                break;
        }
    }

    private void cancelGoal()
    {
        is_finished = true;
        tunnel_interface.cancelGoal();
    }
    private void setFinished() {
        is_finished = true;
    }

    @Override
    public boolean isFinished() {
        return is_finished;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("WaitForCoprocessorPlan finished.");
        if (interrupted) {
            System.out.println("WaitForCoprocessorPlan was interrupted. Cancelling goal.");
            cancelGoal();
        }
        drive.stop();
    }
}
