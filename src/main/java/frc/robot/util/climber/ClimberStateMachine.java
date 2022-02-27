package frc.robot.util.climber;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import frc.robot.subsystems.Climber;

public class ClimberStateMachine {

    private final List<ClimberState> m_states;
    private int m_currentState;

    private ClimberState m_initialState;

    private final double PIVOT_TOLERANCE = 2;
    private final double TELESCOPE_TOLERANCE = 1;

    public ClimberStateMachine(ClimberState firstState) {
        m_states = new ArrayList<>();
        m_states.add(firstState);
        m_currentState = 0;
    }

    private ClimberStateMachine(List<ClimberState> states) {
        m_states = states;
    }

    public ClimberStateMachine addState(ClimberState state) {
        List<ClimberState> newStates = new ArrayList<>(m_states);
        newStates.add(state);
        return new ClimberStateMachine(newStates);
    }

    public ClimberStateMachine addStateMachine(ClimberStateMachine additionalMachine) {
        List<ClimberState> newStates = new ArrayList<>(m_states);
        newStates.addAll(additionalMachine.m_states);
        return new ClimberStateMachine(newStates);
    }

    public void run(Climber climber) {
        ClimberState previousState;
        if (m_currentState == 0 || (m_currentState >= 1 && m_states.size() == 1)) {
            if (Objects.isNull(m_initialState)) {
                m_initialState = new ClimberState(climber.getAverageOuterPivotAngle(), 
                        climber.getAverageOuterTelescopeHeight(), 
                        climber.getAverageInnerPivotAngle(), 
                        climber.getAverageInnerTelescopeHeight());
            }
            previousState = m_initialState;
        } else if (m_currentState >= m_states.size()) {
            previousState = m_states.get(m_states.size() - 2);
        } else {
            previousState = m_states.get(m_currentState - 1);
        }

        if (m_currentState >= m_states.size()) {
            executeState(m_states.get(m_states.size() - 1), previousState, climber);
        } else {
            executeState(m_states.get(m_currentState), previousState, climber);

            if (onTarget(m_states.get(m_currentState), climber)) {
                m_currentState++;
            }
        }
    }

    public boolean isFinished(Climber climber) {
        return m_currentState >= m_states.size();
    }

    public void reset() {
        m_currentState = 0;
        m_initialState = null;
    }

    private void executeState(ClimberState state, ClimberState previousState, Climber climber) {
        if (state.isSynchronized()) {
            double[] durations = new double[] {
                Math.abs(state.getOuterPivot() - previousState.getOuterPivot()) / ClimberArm.getPivotMaxVelocity(),
                Math.abs(state.getOuterTelescope() - previousState.getOuterTelescope()) / ClimberArm.getTelescopeMaxVelocity(),
                Math.abs(state.getInnerPivot() - previousState.getInnerPivot()) / ClimberArm.getPivotMaxVelocity(),
                Math.abs(state.getInnerTelescope() - previousState.getInnerTelescope()) / ClimberArm.getTelescopeMaxVelocity(),
            };

            int maxDurationIndex = 0;
            for (int i = 1; i < durations.length; i++) {
                if (durations[i] > durations[maxDurationIndex]) {
                    maxDurationIndex = i;
                }
            }

            climber.setOuterMotionMagic(
                state.getOuterPivot(),
                state.getOuterTelescope(),
                ClimberArm.getPivotMaxVelocity() * durations[0] / durations[maxDurationIndex], 
                ClimberArm.getTelescopeMaxVelocity() * durations[1] / durations[maxDurationIndex]);
            climber.setOuterMotionMagic(
                state.getOuterPivot(),
                state.getOuterTelescope(),
                ClimberArm.getPivotMaxVelocity() * durations[0] / durations[maxDurationIndex], 
                ClimberArm.getTelescopeMaxVelocity() * durations[1] / durations[maxDurationIndex]);
        }
        climber.setOuterMotionMagic(state.getOuterPivot(), state.getOuterTelescope());
        climber.setInnerMotionMagic(state.getInnerPivot(), state.getInnerTelescope());
    }

    private boolean onTarget(ClimberState state, Climber climber) {
        return Math.abs(state.getOuterPivot() - climber.getAverageOuterPivotAngle()) < PIVOT_TOLERANCE
                && Math.abs(state.getOuterTelescope() - climber.getAverageOuterTelescopeHeight()) < TELESCOPE_TOLERANCE
                && Math.abs(state.getInnerPivot() - climber.getAverageInnerPivotAngle()) < PIVOT_TOLERANCE
                && Math.abs(state.getInnerTelescope() - climber.getAverageInnerTelescopeHeight()) < TELESCOPE_TOLERANCE;
    }
}
