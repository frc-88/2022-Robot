// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.ThisRobotTable;
import frc.robot.util.sensors.Limelight;

public class CollectLimelightRectification extends CommandBase {

  private Limelight m_limelight;
  private ThisRobotTable m_ros;

  private int m_runCounter = 0;

  public CollectLimelightRectification(Limelight limelight, ThisRobotTable ros) {
    m_limelight = limelight;
    m_ros = ros;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("@Begin rectification@");
    m_runCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_runCounter++ % 10 == 0) {
      System.out.println("@" + m_limelight.getTargetDistance() + "," + m_ros.getShooterDistance() + "@");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("@End rectification@");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
