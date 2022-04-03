// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Navigation.RosAutoState;
import frc.robot.util.roswaypoints.Waypoint;
import frc.robot.util.roswaypoints.WaypointsPlan;
import frc.robot.util.coprocessortable.VelocityCommand;

public class DriveToWaypointWithHeading extends CommandBase {
  private final Navigation m_nav;
  private final Drive m_drive;
  private WaypointsPlan m_plan;
  private long m_is_finished_timeout = 0;

  /** Creates a new DriveToWaypointWithHeading. */
  public DriveToWaypointWithHeading(Navigation nav, Drive drive, String waypointName, long is_finished_timeout) {
    m_nav = nav;
    m_drive = drive;
    m_plan = new WaypointsPlan(nav.getCoprocessorTable());
    m_plan.addWaypoint(new Waypoint(waypointName).makeIgnoreOrientation(false));
    m_is_finished_timeout = is_finished_timeout;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(nav);
    addRequirements(drive);
  }
  public DriveToWaypointWithHeading(Navigation nav, Drive drive, String waypointName) {
    this(nav, drive, waypointName, 15_000_000);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_nav.setWaypointsPlan(m_plan, m_is_finished_timeout);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    VelocityCommand command = m_nav.getAutoCommand();
    if (Objects.isNull(command)) {
      return;
    }
    m_drive.drive(command);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_nav.cancelAutoGoal();  // Confirm whether cancelGoal should be called in all cases
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_nav.getRosAutoState() == RosAutoState.FINISHED;
  }
}
