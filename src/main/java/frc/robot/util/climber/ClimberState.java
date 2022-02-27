package frc.robot.util.climber;

public class ClimberState {
    private double m_outerPivot;
    private double m_outerTelescope;
    private double m_innerPivot;
    private double m_innerTelescope;
    private boolean m_synchronize;

    public ClimberState(double outerPivot, double outerTelescope, double innerPivot, double innerTelescope, boolean synchronize) {
        m_outerPivot = outerPivot;
        m_outerTelescope = outerTelescope;
        m_innerPivot = innerPivot;
        m_innerTelescope = innerTelescope;
        m_synchronize = synchronize;
    }

    public ClimberState(double outerPivot, double outerTelescope, double innerPivot, double innerTelescope) {
        this(outerPivot, outerTelescope, innerPivot, innerTelescope, false);
    }

    public double getOuterPivot() {
        return m_outerPivot;
    }

    public double getOuterTelescope() {
        return m_outerTelescope;
    }

    public double getInnerPivot() {
        return m_innerPivot;
    }

    public double getInnerTelescope() {
        return m_innerTelescope;
    }

    public boolean isSynchronized() {
        return m_synchronize;
    }
}
