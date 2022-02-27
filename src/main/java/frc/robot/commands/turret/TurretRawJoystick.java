package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.util.controllers.XboxController;
import frc.robot.util.drive.DriveUtils;

public class TurretRawJoystick extends CommandBase {
    
    private Turret m_turret;
    private XboxController m_controller;

    public TurretRawJoystick(Turret turret, XboxController controller) {
        m_turret = turret;
        m_controller = controller;
        addRequirements(m_turret);
    }

    @Override
    public void execute() {
        m_turret.setPercentOutput(DriveUtils.deadbandExponential(m_controller.getLeftStickX(), 3, 0.25));
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.setPercentOutput(0.0);
    }
}
