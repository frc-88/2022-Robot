package frc.robot.util.climber;

public class ClimberState {
    private final double m_outerPivot;
    private final double m_outerTelescope;
    private final double m_innerPivot;
    private final double m_innerTelescope;
    private final double m_pivotTolerance;
    private final double m_telescopeTolerance;
    private final boolean m_synchronize;
    private final double m_pause;

    public static final double DEFAULT_PIVOT_TOLERANCE = 2;
    public static final double DEFAULT_TELESCOPE_TOLERANCE = 1;

    public ClimberState(
            double outerPivot, 
            double outerTelescope, 
            double innerPivot, 
            double innerTelescope, 
            double pivotTolerance, 
            double telescopeTolerance, 
            boolean synchronize) {

        m_outerPivot = outerPivot;
        m_outerTelescope = outerTelescope;
        m_innerPivot = innerPivot;
        m_innerTelescope = innerTelescope;
        m_pivotTolerance = pivotTolerance;
        m_telescopeTolerance = telescopeTolerance;
        m_synchronize = synchronize;
        m_pause = -1;
        
    }

    public ClimberState(double outerPivot, double outerTelescope, double innerPivot, double innerTelescope) {
        this(outerPivot, outerTelescope, innerPivot, innerTelescope, DEFAULT_PIVOT_TOLERANCE, DEFAULT_TELESCOPE_TOLERANCE, false);
    }

    public ClimberState(double pause) {
        m_outerPivot = 0;
        m_outerTelescope = 0;
        m_innerPivot = 0;
        m_innerTelescope = 0;
        m_pivotTolerance = 0;
        m_telescopeTolerance = 0;
        m_synchronize = false;
        m_pause = pause;
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

    public double getPivotTolerance() {
        return m_pivotTolerance;
    }

    public double getTelescopeTolerance() {
        return m_telescopeTolerance;
    }

    public boolean isSynchronized() {
        return m_synchronize;
    }

    public boolean isPause() {
        return m_pause > 0;
    }

    public double getPause() {
        return m_pause;
    }
}
