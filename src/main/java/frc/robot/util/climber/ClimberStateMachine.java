package frc.robot.util.climber;

import java.util.ArrayList;
import java.util.List;

import frc.robot.subsystems.Climber;

public class ClimberStateMachine {

    public static final ClimberState STOWED = new ClimberState(29, 27, 29, 27);
    
    private final Climber m_climber;

    private final List<ClimberState> m_states;
    private int m_currentState;

    private final double PIVOT_TOLERANCE = 2;
    private final double TELESCOPE_TOLERANCE = 1;

    public ClimberStateMachine(ClimberState firstState, Climber climber) {
        m_climber = climber;
        m_states = new ArrayList<>();
        m_states.add(firstState);
        m_currentState = 0;
    }

    private ClimberStateMachine(List<ClimberState> states, Climber climber) {
        m_climber = climber;
        m_states = states;
    }

    public ClimberStateMachine addState(ClimberState state) {
        List<ClimberState> newStates = new ArrayList<>(m_states);
        newStates.add(state);
        return new ClimberStateMachine(newStates, m_climber);
    }

    public ClimberStateMachine addStateMachine(ClimberStateMachine additionalMachine) {
        List<ClimberState> newStates = new ArrayList<>(m_states);
        newStates.addAll(additionalMachine.m_states);
        return new ClimberStateMachine(newStates, m_climber);
    }

    public void run() {
        if (m_currentState >= m_states.size()) {
            executeState(m_states.get(m_states.size() - 1));
        } else {
            if (onTarget(m_states.get(m_currentState))) {
                m_currentState++;
            }

            executeState(m_states.get(m_currentState));
        }
    }

    public boolean isFinished() {
        return m_currentState >= m_states.size();
    }

    public void reset() {
        m_currentState = 0;
    }

    public Climber getClimber() {
        return m_climber;
    }

    private void executeState(ClimberState state) {
        m_climber.setOuterMotionMagic(state.getOuterPivot(), state.getOuterTelescope());
        m_climber.setInnerMotionMagic(state.getInnerPivot(), state.getInnerTelescope());
    }

    private boolean onTarget(ClimberState state) {
        return Math.abs(state.getOuterPivot() - m_climber.getAverageOuterPivotAngle()) < PIVOT_TOLERANCE
                && Math.abs(state.getOuterTelescope() - m_climber.getAverageOuterTelescopeHeight()) < TELESCOPE_TOLERANCE
                && Math.abs(state.getInnerPivot() - m_climber.getAverageInnerPivotAngle()) < PIVOT_TOLERANCE
                && Math.abs(state.getInnerTelescope() - m_climber.getAverageInnerTelescopeHeight()) < TELESCOPE_TOLERANCE;
    }
}
