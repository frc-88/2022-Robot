// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Sensors;
import frc.robot.util.climber.ClimberStateMachine;

public class ClimberStateMachineExecutor extends CommandBase {
  
  private final Climber m_climber;
  private final Sensors m_sensors;
  private final ClimberStateMachine m_stateMachine;
  private final boolean m_cancelIfRolled;

  private final double ROLL_THRESHOLD = 10;
  private final int ROLL_DURATION = 25;
  private int m_rollCount;
  private boolean m_rolled;

  public ClimberStateMachineExecutor(Climber climber, Sensors sensors, ClimberStateMachine stateMachine, boolean cancelIfRolled) {
    m_climber = climber;
    m_sensors = sensors;
    m_stateMachine = stateMachine;
    m_cancelIfRolled = cancelIfRolled;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    m_stateMachine.reset();
    m_rollCount = 0;
    m_rolled = false;
  }

  @Override
  public void execute() {
    if (Math.abs(m_sensors.navx.getPitch()) > ROLL_THRESHOLD) {
      m_rollCount++;
    } else {
      m_rollCount = 0;
    }

    if (m_cancelIfRolled && m_rollCount > ROLL_DURATION) {
      m_rolled = true;
    }
 
    if (m_rolled) { 
      m_climber.setInnerPercentOutput(0, 0);
      m_climber.setOuterPercentOutput(0, 0);
      return;
    }

    m_stateMachine.run(m_climber);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
