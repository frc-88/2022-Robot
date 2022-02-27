// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Coprocessor;
import frc.robot.subsystems.Coprocessor.RosAutoState;
import frc.robot.util.roswaypoints.WaypointsPlan;

public class DriveToWaypoint extends CommandBase {
  private Coprocessor m_coprocessor;
  private WaypointsPlan m_plan;
  private long m_is_finished_timeout = 0;

  /** Creates a new DriveToWaypoint. */
  public DriveToWaypoint(Coprocessor coprocessor, WaypointsPlan plan, long is_finished_timeout) {
    m_coprocessor = coprocessor;
    m_plan = plan;
    m_is_finished_timeout = is_finished_timeout;
    addRequirements(m_coprocessor);
    addRequirements(m_coprocessor.getDriveSubsystem());
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public DriveToWaypoint(Coprocessor coprocessor, WaypointsPlan plan) {
    this(coprocessor, plan, 15_000_000);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_coprocessor.setWaypointsPlan(m_plan, m_is_finished_timeout);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coprocessor.cancelAutoGoal();  // Confirm whether cancelGoal should be called in all cases
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_coprocessor.getRosAutoState() == RosAutoState.FINISHED;
  }
}
