// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Drive;

public class AutoFollowTrajectory extends CommandBase {
  private Drive m_drive;
  private Trajectory m_trajectory;
  private boolean m_resetOdometry;
  private RamseteController m_controller = new RamseteController();
  private Timer m_timer = new Timer();
  private double m_duration;
  private int m_state;

  public AutoFollowTrajectory(final Drive drive, Trajectory trajectory, boolean resetOdometry) {
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
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    switch (m_state) {
      case 0: // Zero drive
        if (m_resetOdometry) {
          m_drive.resetTrajectoryPose(m_trajectory.getInitialPose());
        }
        m_drive.setBrakeMode();
        m_drive.shiftToHigh();
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
          ChassisSpeeds targetSpeeds = m_controller.calculate(m_drive.getCurrentPose(), goal);

          DifferentialDriveWheelSpeeds wheelSpeeds = m_drive.wheelSpeedsFromChassisSpeeds(targetSpeeds);
          leftSpeed = Units.metersToFeet(wheelSpeeds.leftMetersPerSecond);
          rightSpeed = Units.metersToFeet(wheelSpeeds.rightMetersPerSecond);
        } else {
          m_state++;
        }
        break;

      case 3: // keep updateing the odometry until we have stopped
        if ((Math.abs(m_drive.getLeftSpeed()) < 0.1) && (Math.abs(m_drive.getRightSpeed()) < 0.1)) {
          m_state++;
        }
      default:
        break;
    }
    m_drive.basicDriveLimited(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.basicDriveLimited(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_state > 3;
  }
}
