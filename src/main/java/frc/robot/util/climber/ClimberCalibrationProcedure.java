package frc.robot.util.climber;

import java.util.LinkedList;
import java.util.Queue;

public class ClimberCalibrationProcedure {
    
    private final ClimberArm m_arm;

    private static enum State {
        UNCALIBRATED,
        PUSHING_PIVOT,
        PUSHING_TELESCOPE,
        CALIBRATED;
    }
    private State m_state;

    private Queue<Double> m_pivotPositions;
    private Queue<Double> m_telescopePositions;

    private static final double PIVOT_CALIBRATION_OUTPUT = 0.2;
    private static final double TELESCOPE_CALIBRATION_OUTPUT = -0.1;

    private static final double PIVOT_POSITION_EPSILON = 0.5;
    private static final double TELESCOPE_POSITION_EPSILON = 0.25;

    private static final int POSITIONS_TO_COLLECT = 40;

    public ClimberCalibrationProcedure(ClimberArm arm) {
        m_arm = arm;

        m_pivotPositions = new LinkedList<>();
        m_telescopePositions = new LinkedList<>();

        resetCalibration();
    }

    public void runCalibration() {
        switch (m_state) {
            case UNCALIBRATED:
            case PUSHING_PIVOT:
                m_arm.setPercentOutput(PIVOT_CALIBRATION_OUTPUT, 0);

                double currentPivotPosition = m_arm.getPivotAngle();
                if (m_pivotPositions.size() >= POSITIONS_TO_COLLECT
                        && Math.abs(currentPivotPosition - m_pivotPositions.poll()) < PIVOT_POSITION_EPSILON) {
                    m_state = State.PUSHING_TELESCOPE;
                } else {
                    m_pivotPositions.add(currentPivotPosition);
                    m_state = State.PUSHING_PIVOT;
                }

                break;

            case PUSHING_TELESCOPE:
                m_arm.setPercentOutput(0, TELESCOPE_CALIBRATION_OUTPUT);

                double currentTelescopePosition = m_arm.getTelescopeHeight();
                if (m_telescopePositions.size() >= POSITIONS_TO_COLLECT
                        && Math.abs(currentTelescopePosition - m_telescopePositions.poll()) < TELESCOPE_POSITION_EPSILON) {
                    m_arm.finishCalibration();
                    m_arm.setPercentOutput(0, 0);
                    m_state = State.CALIBRATED;
                } else {
                    m_telescopePositions.add(currentTelescopePosition);
                    m_state = State.PUSHING_TELESCOPE;
                }

                break;

            case CALIBRATED:
                m_arm.setPercentOutput(0, 0);

                m_state = State.CALIBRATED;
                
                break;
        }
    }

    public void resetCalibration() {
        m_state = State.UNCALIBRATED;

        m_pivotPositions.clear();
        m_telescopePositions.clear();

        m_arm.enterCalibration();

    }

    public boolean isCalibrated() {
        return m_state == State.CALIBRATED;
    }

}
