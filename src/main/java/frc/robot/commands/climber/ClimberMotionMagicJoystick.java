package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.util.climber.ClimberArm;
import frc.robot.util.controllers.XboxController;
import frc.robot.util.drive.DriveUtils;

public class ClimberMotionMagicJoystick extends CommandBase {
    
    private Climber m_climber;
    private XboxController m_controller;

    private static final double PIVOT_SPEED = 5;
    private static final double TELESCOPE_SPEED = 2;

    private static final double BUMPER_MULTIPLIER = 20;
    
    public ClimberMotionMagicJoystick(Climber climber, XboxController controller) {
        m_climber = climber;
        m_controller = controller;
        addRequirements(m_climber);
    }

    @Override
    public void execute() {
        double innerPivot = DriveUtils.deadbandExponential(-m_controller.getLeftStickX(), 3, 0.25) * PIVOT_SPEED;
        double innerTelescope = DriveUtils.deadbandExponential(m_controller.getLeftStickY(), 3, 0.25) * TELESCOPE_SPEED;
        double outerPivot = DriveUtils.deadbandExponential(-m_controller.getRightStickX(), 3, 0.25) * PIVOT_SPEED;
        double outerTelescope = DriveUtils.deadbandExponential(m_controller.getRightStickY(), 3, 0.25) * TELESCOPE_SPEED;

        if (m_controller.buttonLeftBumper.get()) {
            innerTelescope *= BUMPER_MULTIPLIER;
            outerTelescope *= BUMPER_MULTIPLIER;
        }

        innerPivot += m_climber.getAverageInnerPivotAngle();
        innerTelescope += m_climber.getAverageInnerTelescopeHeight();
        outerPivot += m_climber.getAverageOuterPivotAngle();
        outerTelescope += m_climber.getAverageOuterTelescopeHeight();

        if (ClimberArm.isPositionInvalid(innerPivot, innerTelescope)) {
            System.out.println("Inner Invalid");
        }
        if (ClimberArm.isPositionInvalid(outerPivot, outerTelescope)) {
            System.out.println("Outer Invalid");
        }

        m_climber.setInnerMotionMagic(innerPivot, innerTelescope);
        m_climber.setOuterMotionMagic(outerPivot, outerTelescope);
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.setInnerMotionMagic(m_climber.getAverageInnerPivotAngle(), m_climber.getAverageInnerTelescopeHeight());
        m_climber.setOuterMotionMagic(m_climber.getAverageOuterPivotAngle(), m_climber.getAverageOuterTelescopeHeight());
    }
}
