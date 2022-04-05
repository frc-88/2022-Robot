// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveDegrees extends CommandBase {
  private Drive m_drive;
  private Pose2d startPose;
  private double angleDegrees;
  private double rotationalVelocityRadiansPerSecond;
  /** Creates a new DriveDegrees. */
  public DriveDegrees(Drive drive, double angleDegrees, double rotationalVelocityDegreesPerSecond) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    this.angleDegrees = angleDegrees;
    this.rotationalVelocityRadiansPerSecond = Units.degreesToRadians(rotationalVelocityDegreesPerSecond);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Running DriveDegrees command");
    startPose = m_drive.getOdometryPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(0.0, 0.0, rotationalVelocityRadiansPerSecond);
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
    double angle = relativePose.getRotation().getDegrees();
    boolean should_stop = false;
    if (rotationalVelocityRadiansPerSecond > 0) {
      if (angle < 0) {
        angle += 360.0;  // put between 0..360 to avoid wrap around issues
      }
      should_stop = angle > angleDegrees;
    }
    else {
      if (angle > 0) {
        angle -= 360.0;  // put between -360..0 to avoid wrap around issues
      }
      should_stop = angle < angleDegrees;
    }
    return should_stop;
  }
}
