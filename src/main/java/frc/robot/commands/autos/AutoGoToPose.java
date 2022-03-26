// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.util.RapidReactTrajectories;

public class AutoGoToPose extends CommandBase {
  private Drive m_drive;
  private Pose2d m_targetPose;
  private boolean m_reverse;
  private Trajectory m_trajectory;
  private RamseteController m_controller = new RamseteController();
  private Timer m_trajectoryTimer = new Timer();
  private double m_duration;
  private int m_state;

  /** Creates a new AutoGoToPose. */
  public AutoGoToPose(Drive drive, Pose2d pose, boolean reverse) {
    m_drive = drive;
    m_targetPose = pose;
    m_reverse = reverse;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TrajectoryConfig config = RapidReactTrajectories.basicConfig();
    config.setReversed(m_reverse);
    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(m_drive.getCurrentPose());
    waypoints.add(m_targetPose);
    
    Timer generationTimer = new Timer();
    generationTimer.reset();
    generationTimer.start();
    m_trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);
    generationTimer.stop();
    System.out.println("AutoGoToPose generation time:" + generationTimer.get() + "(s)");

    m_duration = m_trajectory.getTotalTimeSeconds();
    m_state = 0;
    m_trajectoryTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    switch (m_state) {
      case 0: // Prep, reset the timer and go!
        m_drive.setBrakeMode();
        m_drive.shiftToHigh();
        m_trajectoryTimer.start();
        m_state++;
        // go to state 1 right away

      case 1: // follow the trajectory for its duration
        if (m_trajectoryTimer.get() < m_duration) {
          double now = m_trajectoryTimer.get();
          Trajectory.State goal = m_trajectory.sample(now);
          ChassisSpeeds targetSpeeds = m_controller.calculate(m_drive.getCurrentPose(), goal);

          DifferentialDriveWheelSpeeds wheelSpeeds = m_drive.wheelSpeedsFromChassisSpeeds(targetSpeeds);
          leftSpeed = Units.metersToFeet(wheelSpeeds.leftMetersPerSecond);
          rightSpeed = Units.metersToFeet(wheelSpeeds.rightMetersPerSecond);
        } else {
          m_state++;
        }
        break;

      case 2: // keep going until we stop
        if ((Math.abs(m_drive.getLeftSpeed()) < 0.1) && (Math.abs(m_drive.getRightSpeed()) < 0.1)) {
          m_state++;
        }
        break;

      default:
        // shouldn't get here, end the command if we do
        m_state = 99;
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
    return m_state > 2;
  }
}
