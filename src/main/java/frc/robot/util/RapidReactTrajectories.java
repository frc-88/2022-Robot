// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;

public class RapidReactTrajectories
{
  public RapidReactTrajectories() {
  }

  public static TrajectoryConfig basicConfig() {
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
 
  public static Trajectory generateStraightTrajectory(double distanceFeet) {
    TrajectoryConfig config = basicConfig();
    
    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0), new Rotation2d()));
    waypoints.add(new Pose2d(Units.feetToMeters(distanceFeet), Units.feetToMeters(0.0), new Rotation2d()));

    return TrajectoryGenerator.generateTrajectory(waypoints, config);
  }

  public static Trajectory generatePathWeaverTrajectory(String trajectoryJSON) {
    trajectoryJSON = "output/" + trajectoryJSON;
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    return trajectory;
  }

}
