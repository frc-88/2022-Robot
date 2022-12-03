// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.coprocessor.GameObject;
import frc.robot.util.coprocessor.VelocityCommand;
import frc.robot.util.coprocessor.networktables.CoprocessorTable;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class ChaseObject extends CommandBase {
  private final SwerveDrive m_drive;
  private final CoprocessorTable m_coprocessor;
  private String m_objectName = "";
  private PIDController m_controller;
  private double m_maxVelocity = 2.0;
  private double m_maxAngularVelocity = 6.0;
  private double m_goalDistance = Double.NaN;
  private double m_distanceThreshold = 1.0;
  private double k_distanceRamp = 2.0;
  private double kP = 2.3;
  private double kD = 0.1;
  private double kI = 0.01;

  private DoublePreferenceConstant p_kP = new DoublePreferenceConstant("ROS Chase kP", kP);
  private DoublePreferenceConstant p_kD = new DoublePreferenceConstant("ROS Chase kD", kD);
  private DoublePreferenceConstant p_kI = new DoublePreferenceConstant("ROS Chase kI", kI);
  private DoublePreferenceConstant p_chase_maxVelocity = new DoublePreferenceConstant("ROS Chase Max Velocity", m_maxVelocity);
  private DoublePreferenceConstant p_maxAngularVelocity = new DoublePreferenceConstant("ROS Chase Max Angular Velocity", m_maxAngularVelocity);
  private DoublePreferenceConstant p_distanceThreshold = new DoublePreferenceConstant("ROS Chase Distance Threshold", m_distanceThreshold);

  /** Creates a new ChaseObject. */
  public ChaseObject(
      SwerveDrive drive, CoprocessorTable coprocessor, String objectName) {
    m_drive = drive;
    m_coprocessor = coprocessor;
    m_objectName = objectName;

    m_controller = new PIDController(kP, kI, kD);
    m_controller.setSetpoint(0.0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP = p_kP.getValue();
    kI = p_kD.getValue();
    kD = p_kI.getValue();
    m_maxVelocity = p_chase_maxVelocity.getValue();
    m_maxAngularVelocity = p_maxAngularVelocity.getValue();
    m_distanceThreshold = p_distanceThreshold.getValue();
    m_controller.setP(kP);
    m_controller.setI(kI);
    m_controller.setD(kD);

    m_controller.reset();
    m_goalDistance = Double.NaN;
    System.out.println("Chasing " + m_objectName);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    GameObject gameObject = m_coprocessor.getNearestGameObject(m_objectName);
    if (!gameObject.isValid()) {
      m_drive.stop();
      System.out.println(String.format("Requested game object %s is not available", m_objectName));
      return;
    }
    
    Pose2d relativeGoalPose = gameObject.getPose();

    m_goalDistance = relativeGoalPose.getX();  // ignoring Y component since we don't directly control it here
    double error = new Rotation2d(relativeGoalPose.getX(), relativeGoalPose.getY()).getRadians();
    error *= -1.0;
    
    VelocityCommand driveCommand = new VelocityCommand();
    driveCommand.vt = m_controller.calculate(error);
    if (Math.abs(driveCommand.vt) > m_maxAngularVelocity) {
      driveCommand.vt = Math.copySign(m_maxAngularVelocity, driveCommand.vt);
    }
    
    driveCommand.vx = m_goalDistance * k_distanceRamp;
    if (Math.abs(driveCommand.vx) > m_maxVelocity) {
      driveCommand.vx = Math.copySign(m_maxVelocity, driveCommand.vx);
    }

    driveCommand.vy = 0.0;

    ChassisSpeeds speeds = m_drive.getChassisSpeeds();
    double speed = Math.sqrt(
      speeds.vxMetersPerSecond * speeds.vxMetersPerSecond + 
      speeds.vyMetersPerSecond * speeds.vyMetersPerSecond
    );
    if (m_coprocessor.getLaserScanObstacles().isObstacleWithinBounds(speed)) {
      System.out.println("Obstacle detected within bounds!");
      if (m_coprocessor.getLaserScanObstacles().isDirectionAllowed(driveCommand.getHeading(), speed)) {
        m_drive.drive(driveCommand);
      } else {
        System.out.println("Velocity command doesn't move robot away from obstacle! Ignoring.");
        m_drive.stop();
      }
    } else {
      m_drive.drive(driveCommand);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Finished chasing " + m_objectName);
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !Double.isNaN(m_goalDistance) && Math.abs(m_goalDistance) < m_distanceThreshold;
  }
}
