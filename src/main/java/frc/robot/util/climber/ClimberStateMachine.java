package frc.robot.util.climber;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.Climber;

public class ClimberStateMachine {

    private final List<ClimberState> m_states;
    private int m_currentState;

    private ClimberState m_initialState;

    private double m_startPauseTime = -1;

    public ClimberStateMachine(ClimberState firstState) {
        m_states = new ArrayList<>();
        m_states.add(firstState);
        m_currentState = -1;
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
        if (m_currentState <= 0) {
            if (Objects.isNull(m_initialState)) {
                m_initialState = new ClimberState(climber.getAverageOuterPivotAngle(), 
                        climber.getAverageOuterTelescopeHeight(), 
                        climber.getAverageInnerPivotAngle(), 
                        climber.getAverageInnerTelescopeHeight());
            }
            previousState = m_initialState;
        } else if (m_currentState < m_states.size()) {
            previousState = m_states.get(m_currentState - 1);
        } else {
            previousState = m_states.get(0);
        }

        if (m_currentState < 0 || m_currentState < (m_states.size() - 1) && onTarget(m_states.get(m_currentState), climber)) {
            m_currentState++;
            executeState(m_states.get(m_currentState), previousState, climber);
        }
    }

    public boolean isFinished(Climber climber) {
        return m_currentState >= m_states.size();
    }

    public void reset() {
        m_currentState = -1;
        m_initialState = null;
    }

    private void executeState(ClimberState state, ClimberState previousState, Climber climber) {
        if (state.isPause()) {
            m_startPauseTime = RobotController.getFPGATime() / 1_000_000;
            return;
        }

        if (state.isSynchronized()) {
            double pivotMaxVelocity = ClimberArm.getPivotMaxVelocity();
            double telescopeMaxVelocity = ClimberArm.getTelescopeMaxVelocity();

            double[] durations = new double[] {
                Math.abs(state.getOuterPivot() - previousState.getOuterPivot()) / pivotMaxVelocity,
                Math.abs(state.getOuterTelescope() - previousState.getOuterTelescope()) / telescopeMaxVelocity,
                Math.abs(state.getInnerPivot() - previousState.getInnerPivot()) / pivotMaxVelocity,
                Math.abs(state.getInnerTelescope() - previousState.getInnerTelescope()) / telescopeMaxVelocity,
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
                pivotMaxVelocity * durations[0] / durations[maxDurationIndex], 
                telescopeMaxVelocity * durations[1] / durations[maxDurationIndex]);
            climber.setInnerMotionMagic(
                state.getInnerPivot(),
                state.getInnerTelescope(),
                pivotMaxVelocity * durations[2] / durations[maxDurationIndex], 
                telescopeMaxVelocity * durations[3] / durations[maxDurationIndex]);
        } else {
            climber.setOuterMotionMagic(state.getOuterPivot(), state.getOuterTelescope());
            climber.setInnerMotionMagic(state.getInnerPivot(), state.getInnerTelescope());
        }
    }

    private boolean onTarget(ClimberState state, Climber climber) {
        if (state.isPause()) {
            return RobotController.getFPGATime() / 1_000_000. - m_startPauseTime >= state.getPause();
        }

        return Math.abs(state.getOuterPivot() - climber.getAverageOuterPivotAngle()) < state.getPivotTolerance()
                && Math.abs(state.getOuterTelescope() - climber.getAverageOuterTelescopeHeight()) < state.getTelescopeTolerance()
                && Math.abs(state.getInnerPivot() - climber.getAverageInnerPivotAngle()) < state.getPivotTolerance()
                && Math.abs(state.getInnerTelescope() - climber.getAverageInnerTelescopeHeight()) < state.getTelescopeTolerance();
    }
}
