package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.util.climber.ClimberArm;

public class ClimberTestMotionMagic extends CommandBase {

    private Climber m_climber;
    
    public ClimberTestMotionMagic(Climber climber) {
        m_climber = climber;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Inner Climber Set Pivot", m_climber.getAverageInnerPivotAngle());
        SmartDashboard.putNumber("Inner Climber Set Telescope", m_climber.getAverageInnerTelescopeHeight());
        SmartDashboard.putNumber("Outer Climber Set Pivot", m_climber.getAverageOuterPivotAngle());
        SmartDashboard.putNumber("Outer Climber Set Telescope", m_climber.getAverageOuterTelescopeHeight());
    }

    @Override
    public void execute() {
        double innerPivot = SmartDashboard.getNumber("Inner Climber Set Pivot", m_climber.getAverageInnerPivotAngle());
        double innerTelescope = SmartDashboard.getNumber("Inner Climber Set Telescope", m_climber.getAverageInnerTelescopeHeight());
        double outerPivot = SmartDashboard.getNumber("Outer Climber Set Pivot", m_climber.getAverageOuterPivotAngle());
        double outerTelescope = SmartDashboard.getNumber("Outer Climber Set Telescope", m_climber.getAverageOuterTelescopeHeight());

        if (!ClimberArm.isPositionInvalid(innerPivot, innerTelescope)) {
            m_climber.setInnerMotionMagic(innerPivot, innerTelescope);
        } else {
            System.err.println("Invalid inner climber position: " + innerPivot + " degrees, " + innerTelescope + " inches");
        }
        if (!ClimberArm.isPositionInvalid(outerPivot, outerTelescope)) {
            m_climber.setOuterMotionMagic(outerPivot, outerTelescope);
        } else {
            System.err.println("Invalid outer climber position: " + outerPivot + " degrees, " + outerTelescope + " inches");
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.delete("Inner Climber Set Pivot");
        SmartDashboard.delete("Inner Climber Set Telescope");
        SmartDashboard.delete("Outer Climber Set Pivot");
        SmartDashboard.delete("Outer Climber Set Telescope");
    }

}
