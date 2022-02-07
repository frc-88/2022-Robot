package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.util.TJController;
import frc.robot.util.drive.DriveUtils;

public class ClimberMotionMagicJoystick extends CommandBase {
    
    private Climber m_climber;
    private TJController m_controller;

    private static final double PIVOT_SPEED = 5;
    private static final double TELESCOPE_SPEED = 2;

    private static final double BUMPER_MULTIPLIER = 5;
    
    public ClimberMotionMagicJoystick(Climber climber, TJController controller) {
        m_climber = climber;
        m_controller = controller;
        addRequirements(m_climber);
    }

    @Override
    public void execute() {
        double leftStickX = DriveUtils.deadbandExponential(m_controller.getLeftStickX(), 3, 0.25);
        double leftStickY = DriveUtils.deadbandExponential(m_controller.getLeftStickY(), 3, 0.25);
        double rightStickX = DriveUtils.deadbandExponential(m_controller.getRightStickX(), 3, 0.25);
        double rightStickY = DriveUtils.deadbandExponential(m_controller.getRightStickY(), 3, 0.25);

        if (m_controller.buttonRightBumper.get()) {
            leftStickY *= BUMPER_MULTIPLIER;
            rightStickY *= BUMPER_MULTIPLIER;
        }

        m_climber.setInnerMotionMagic(m_climber.getAverageInnerPivotAngle() + leftStickX * PIVOT_SPEED, m_climber.getAverageInnerTelescopeHeight() + leftStickY * TELESCOPE_SPEED);
        m_climber.setOuterMotionMagic(m_climber.getAverageOuterPivotAngle() + rightStickX * PIVOT_SPEED, m_climber.getAverageOuterTelescopeHeight() + rightStickY * TELESCOPE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.setInnerMotionMagic(m_climber.getAverageInnerPivotAngle(), m_climber.getAverageInnerTelescopeHeight());
        m_climber.setOuterMotionMagic(m_climber.getAverageOuterPivotAngle(), m_climber.getAverageOuterTelescopeHeight());
    }
}
