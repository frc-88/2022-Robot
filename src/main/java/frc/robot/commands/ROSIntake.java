// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.util.coprocessor.GameObject;
import frc.robot.util.coprocessor.networktables.CoprocessorTable;

public class ROSIntake extends CommandBase {
  private CoprocessorTable m_coprocessor;
  private String m_objectName = "";
  private Intake m_intake;
  private double m_distanceThreshold = 1; //measured in meters

  /** Creates a new ChaseObject. */
  public ROSIntake(
    Intake intake, CoprocessorTable coprocessor, String objectName, double distanceThreshold) {
    m_intake = intake;
    m_coprocessor = coprocessor;
    m_objectName = objectName;
    m_distanceThreshold = distanceThreshold;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    GameObject gameObject = m_coprocessor.getNearestGameObject(m_objectName);
    if (!gameObject.isValid()) {
      System.out.println(String.format("Requested game object %s is not available", m_objectName));
      stowIntake();
      return;
    }

    Pose2d relativeGoalPose = gameObject.getPose();
    
    double goalDistance = relativeGoalPose.getX();  // ignoring Y component since we don't directly control it here
    if (goalDistance < m_distanceThreshold) {
        deployIntake();
    }
    else {
        stowIntake();
    }
  }
  
  private void deployIntake () {
    m_intake.deploy();
    m_intake.rollerIntake();
  }
  
  private void stowIntake() {
        m_intake.stow();
        m_intake.rollerStop();
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Finished chasing " + m_objectName);
    stowIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}