package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

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
        m_climber.setInnerMotionMagic(
            SmartDashboard.getNumber("Inner Climber Set Pivot", m_climber.getAverageInnerPivotAngle()),
            SmartDashboard.getNumber("Inner Climber Set Telescope", m_climber.getAverageInnerTelescopeHeight()));
        m_climber.setOuterMotionMagic(
            SmartDashboard.getNumber("Outer Climber Set Pivot", m_climber.getAverageOuterPivotAngle()),
            SmartDashboard.getNumber("Outer Climber Set Telescope", m_climber.getAverageOuterTelescopeHeight()));
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.delete("Inner Climber Set Pivot");
        SmartDashboard.delete("Inner Climber Set Telescope");
        SmartDashboard.delete("Outer Climber Set Pivot");
        SmartDashboard.delete("Outer Climber Set Telescope");
    }

}
