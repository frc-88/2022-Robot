// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class RapidReactTrajectories
{
  // CARGO starting locations for reference
  // constants indicate starting locations on the respective color side of the field
  // index numbers go clockwise (right to left from driver station view)
  // TODO real numbers for below if we ever feel the need to have this information
  // private static final Translation2d RED_1 =  new Translation2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0)); // Blue CARGO
  // private static final Translation2d RED_2 =  new Translation2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0)); // Red CARGO
  // private static final Translation2d RED_3 =  new Translation2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0)); // Red CARGO
  // private static final Translation2d RED_4 =  new Translation2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0)); // Blue CARGO
  // private static final Translation2d RED_5 =  new Translation2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0)); // Red CARGO
  // private static final Translation2d RED_6 =  new Translation2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0)); // Blue CARGO

  // private static final Translation2d BLUE_1 =  new Translation2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0)); // Red CARGO
  // private static final Translation2d BLUE_2 =  new Translation2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0)); // Blue CARGO
  // private static final Translation2d BLUE_3 =  new Translation2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0)); // Blue CARGO
  // private static final Translation2d BLUE_4 =  new Translation2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0)); // Red CARGO
  // private static final Translation2d BLUE_5 =  new Translation2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0)); // Blue CARGO
  // private static final Translation2d BLUE_6 =  new Translation2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0)); // Red CARGO

  // private static final Translation2d RED_TERMINAL =  new Translation2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0)); // Red CARGO
  // private static final Translation2d BLUE_TERMINAL =  new Translation2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0)); // Blue CARGO

  public RapidReactTrajectories() {
  }

  private static TrajectoryConfig initializeConfig() {
    TrajectoryConfig config;
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.feetToMeters(Constants.WHEEL_BASE_WIDTH));
    config = new TrajectoryConfig(Units.feetToMeters(Constants.MAX_TRAJ_VELOCITY), Units.feetToMeters(Constants.MAX_TRAJ_ACCELERATION));
    config.setKinematics(kinematics);
    config.setStartVelocity(0.0D);
    config.setEndVelocity(0.0D);
    config.addConstraint((TrajectoryConstraint)new DifferentialDriveKinematicsConstraint(kinematics, Units.feetToMeters(Constants.MAX_TRAJ_VELOCITY)));
    config.addConstraint((TrajectoryConstraint)new CentripetalAccelerationConstraint(Constants.MAX_TRAJ_CENTRIP_ACC));

    return config;
  }
 
  public static Trajectory generateTestTrajectory() {
    TrajectoryConfig config = initializeConfig();

    // drive ten feet forward, keep same facing
    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0), new Rotation2d()));
    waypoints.add(new Pose2d(Units.feetToMeters(10.0), Units.feetToMeters(0.0), new Rotation2d()));
    
    return TrajectoryGenerator.generateTrajectory(waypoints, config);
  }

  // Barrel runs from last year for testing and simulation
  public static Trajectory generateBarrelRunTrajectory() {
    TrajectoryConfig config = initializeConfig();

    Pose2d start = new Pose2d(Units.feetToMeters(5.0D), Units.feetToMeters(7.5D), new Rotation2d());

    ArrayList<Translation2d> waypoints = new ArrayList<>();
    waypoints.add(new Translation2d(Units.feetToMeters(13.5D), Units.feetToMeters(7.5D)));
    waypoints.add(new Translation2d(Units.feetToMeters(16.1D), Units.feetToMeters(4.0D)));
    waypoints.add(new Translation2d(Units.feetToMeters(12.0D), Units.feetToMeters(3.0D)));
    waypoints.add(new Translation2d(Units.feetToMeters(12.5D), Units.feetToMeters(7.5D)));
    waypoints.add(new Translation2d(Units.feetToMeters(21.0D), Units.feetToMeters(7.5D)));
    waypoints.add(new Translation2d(Units.feetToMeters(25.0D), Units.feetToMeters(12.5D)));
    waypoints.add(new Translation2d(Units.feetToMeters(17.8D), Units.feetToMeters(12.8D)));
    waypoints.add(new Translation2d(Units.feetToMeters(25.0D), Units.feetToMeters(2.0D)));
    waypoints.add(new Translation2d(Units.feetToMeters(29.1D), Units.feetToMeters(5.0D)));
    waypoints.add(new Translation2d(Units.feetToMeters(26.0D), Units.feetToMeters(7.8D)));
    waypoints.add(new Translation2d(Units.feetToMeters(15.0D), Units.feetToMeters(8.1D)));

    Pose2d end = new Pose2d(Units.feetToMeters(0.0D), Units.feetToMeters(8.1D), Rotation2d.fromDegrees(180.0D));

    return TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
  }

  public static Trajectory generateBarrelRun2Trajectory() {
    TrajectoryConfig config = initializeConfig();

    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(3.9453125D), Units.feetToMeters(7.5D), new Rotation2d()));
    waypoints.add(new Pose2d(Units.feetToMeters(15.5D), Units.feetToMeters(5.0D), Rotation2d.fromDegrees(-90.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(12.5D), Units.feetToMeters(2.0D), Rotation2d.fromDegrees(180.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(9.5D), Units.feetToMeters(5.0D), Rotation2d.fromDegrees(90.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(23.0D), Units.feetToMeters(10.0D), Rotation2d.fromDegrees(90.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(20.0D), Units.feetToMeters(13.0D), Rotation2d.fromDegrees(180.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(17.0D), Units.feetToMeters(10.0D), Rotation2d.fromDegrees(-90.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(25.0D), Units.feetToMeters(2.0D), new Rotation2d()));
    waypoints.add(new Pose2d(Units.feetToMeters(28.0D), Units.feetToMeters(5.0D), Rotation2d.fromDegrees(90.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(25.0D), Units.feetToMeters(8.0D), Rotation2d.fromDegrees(180.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(0.5D), Units.feetToMeters(7.5D), Rotation2d.fromDegrees(180.0D)));

    return TrajectoryGenerator.generateTrajectory(waypoints, config);
  }

}