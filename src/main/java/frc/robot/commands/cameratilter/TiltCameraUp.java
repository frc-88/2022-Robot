// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.cameratilter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Sensors;

public class TiltCameraUp extends CommandBase {
  private Sensors m_sensors;

  /** Creates a new TiltCameraDown. */
  public TiltCameraUp(Sensors sensors) {
    m_sensors = sensors;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_sensors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_sensors.setCameraTilterAngle(Constants.CAMERA_TILT_DOWN_ANGLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
