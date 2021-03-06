// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Sensors;
import frc.robot.util.climber.ClimberStateMachine;

public class ClimberStateMachineExecutor extends CommandBase {
  
  private final Climber m_climber;
  private final Sensors m_sensors;
  private final ClimberStateMachine m_stateMachine;
  private final boolean m_cancelIfRolled;
  private final BooleanSupplier m_cancelButton;
  private final BooleanSupplier m_pauseButton;

  private final double ROLL_THRESHOLD = 10;
  private final int ROLL_DURATION = 25;
  private int m_rollCount;
  private boolean m_rolled;
  private boolean m_wasPaused;

  public ClimberStateMachineExecutor(Climber climber, Sensors sensors, ClimberStateMachine stateMachine) {
    this(climber, sensors, stateMachine, false, () -> false, () -> false);
  }

  public ClimberStateMachineExecutor(Climber climber, Sensors sensors, ClimberStateMachine stateMachine, boolean cancelIfRolled, BooleanSupplier cancelButton, BooleanSupplier pauseButton) {
    m_climber = climber;
    m_sensors = sensors;
    m_stateMachine = stateMachine;
    m_cancelIfRolled = cancelIfRolled;
    m_cancelButton = cancelButton;
    m_pauseButton = pauseButton;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    m_stateMachine.reset();
    m_rollCount = 0;
    m_rolled = false;
    m_wasPaused = false;
  }

  @Override
  public void execute() {
    if (Math.abs(m_sensors.navx.getPitch()) > ROLL_THRESHOLD) {
      m_rollCount++;
    } else {
      m_rollCount = 0;
    }

    if (m_cancelButton.getAsBoolean() || (m_cancelIfRolled && m_rollCount > ROLL_DURATION)) {
      m_rolled = true;
    }
 
    if (m_rolled || m_pauseButton.getAsBoolean()) { 
      m_climber.setInnerMotionMagic(m_climber.getAverageInnerPivotAngle(), m_climber.getAverageInnerTelescopeHeight());
      m_climber.setOuterMotionMagic(m_climber.getAverageOuterPivotAngle(), m_climber.getAverageOuterTelescopeHeight());
      m_sensors.setCameraTilterAngle(Constants.CAMERA_TILT_DOWN_ANGLE);  // tilt camera to look down at the intake
      return;
    }

    m_sensors.setCameraTilterAngle(Constants.CAMERA_TILT_UP_ANGLE);  // tilt camera to look up at the bars
    m_stateMachine.run(m_climber, m_wasPaused);
    m_wasPaused = m_pauseButton.getAsBoolean();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_sensors.setCameraTilterAngle(Constants.CAMERA_TILT_DOWN_ANGLE);  // tilt camera to look down at the intake
  }
}
