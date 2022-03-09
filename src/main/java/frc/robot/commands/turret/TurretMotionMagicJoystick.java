package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.util.controllers.XboxController;
import frc.robot.util.drive.DriveUtils;

public class TurretMotionMagicJoystick extends CommandBase {
    
    private Turret m_turret;
    private XboxController m_controller;

    private static final double ROTATION_SPEED = 30;
    
    public TurretMotionMagicJoystick(Turret turret, XboxController controller) {
        m_turret = turret;
        m_controller = controller;
        addRequirements(m_turret);
    }

    @Override
    public void execute() {
        double target = DriveUtils.deadbandExponential(m_controller.getLeftStickX(), 3, 0.25) * ROTATION_SPEED;

        target += m_turret.getFacing();

        m_turret.goToFacing(target);
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.setPercentOutput(0.0);
    }
}
