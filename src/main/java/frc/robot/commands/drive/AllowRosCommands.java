// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.util.tunnel.ROSInterface;
import frc.robot.util.tunnel.TunnelServer;

public class AllowRosCommands extends CommandBase {
    private Drive m_drive;
    private ROSInterface m_tunnel_interface;

    /** Creates a new AllowRosCommands. */
    public AllowRosCommands(Drive drive, ROSInterface tunnel_interface) {
        m_drive = drive;
        m_tunnel_interface = tunnel_interface;
        addRequirements(m_drive);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (TunnelServer.anyClientsAlive() && m_tunnel_interface.isCommandActive()) {
            m_drive.drive(m_tunnel_interface.getCommand());
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
