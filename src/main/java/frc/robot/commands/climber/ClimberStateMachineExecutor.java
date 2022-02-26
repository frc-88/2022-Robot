// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.climber.ClimberStateMachine;

public class ClimberStateMachineExecutor extends CommandBase {
  
  private final ClimberStateMachine m_stateMachine;

  public ClimberStateMachineExecutor(ClimberStateMachine stateMachine) {
    m_stateMachine = stateMachine;
    addRequirements(stateMachine.getClimber());
  }

  @Override
  public void initialize() {
    m_stateMachine.reset();
  }

  @Override
  public void execute() {
    m_stateMachine.run();
  }

  @Override
  public boolean isFinished() {
    return m_stateMachine.isFinished();
  }
}
