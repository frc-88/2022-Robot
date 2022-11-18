// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.subsystems.SwerveDrive;

public class FollowTrajectory extends CommandBase {
  private SwerveDrive m_drive;
  private Trajectory m_trajectory;
  private boolean m_resetOdometry;
  private RamseteController m_controller = new RamseteController();
  private Timer m_timer = new Timer();
  private double m_duration;
  private int m_state;

  public FollowTrajectory(final SwerveDrive drive, Trajectory trajectory, boolean resetOdometry) {
    m_drive = drive;
    m_resetOdometry = resetOdometry;
    m_trajectory = trajectory;
    m_duration = m_trajectory.getTotalTimeSeconds();

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state = 0;
    SmartDashboard.putNumber("Auto State", m_state);
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Auto State", m_state);
    ChassisSpeeds targetSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    switch (m_state) {
      case 0: // Zero drive
        if (m_resetOdometry) {
          m_drive.resetTrajectoryPose(m_trajectory.getInitialPose());
          // m_drive.resetOdometry(m_trajectory.getInitialPose(), m_drive.getGyroscopeRotation());
        }
        m_state++;
        break;
      case 1: // reset the timer and go!
        m_timer.start();
        m_state++;
        // fall through right away to case 2
      case 2: // follow the trajectory, our final state
        if (m_timer.get() < m_duration) {
          double now = m_timer.get();
          Trajectory.State goal = m_trajectory.sample(now);
          targetSpeeds = m_controller.calculate(m_drive.getTrajectoryOdometryPose(), goal);
          // targetSpeeds = m_controller.calculate(m_drive.getOdometryPose(), goal);
        } else {
          m_state++;
        }
        break;

      default:
        break;
    }

    m_drive.drive(targetSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_state > 2;
  }
}