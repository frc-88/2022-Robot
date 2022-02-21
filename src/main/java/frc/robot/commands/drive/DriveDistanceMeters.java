// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveDistanceMeters extends CommandBase {
  private Drive m_drive;
  private Pose2d startPose;
  private double distanceMeters;
  private double translationVelocityMetersPerSecond;
  /** Creates a new DriveDistanceMeters. */
  public DriveDistanceMeters(Drive drive, double distanceMeters, double translationVelocityMetersPerSecond) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    this.distanceMeters = distanceMeters;
    this.translationVelocityMetersPerSecond = translationVelocityMetersPerSecond;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Running DriveDistanceMeters command");
    startPose = m_drive.getOdometryPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(translationVelocityMetersPerSecond, 0.0, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d currentPose = m_drive.getOdometryPose();
    Pose2d relativePose = currentPose.relativeTo(startPose);
    System.out.println("Distance: " + relativePose.getX());
    if (relativePose.getX() > distanceMeters) {
      return true;
    }
    else {
      return false;
    }
  }
}
