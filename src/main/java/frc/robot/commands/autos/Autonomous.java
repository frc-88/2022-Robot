// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.cameratilter.TiltCameraDown;
import frc.robot.commands.drive.DriveDistanceMeters;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.util.RapidReactTrajectories;
import frc.robot.util.coprocessortable.CoprocessorTable;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.roswaypoints.Waypoint;
import frc.robot.util.roswaypoints.WaypointsPlan;

/** Add your docs here. */
public class Autonomous {
    public static CommandBase generateOneBall(Drive drive, Navigation nav, Sensors sensors, Shooter shooter, Turret turret, Intake intake) {
    return new SequentialCommandGroup(
      new SetGlobalPoseToWaypoint(nav, "start_" + getTeamColorName()),
      new TiltCameraDown(sensors),
        new InstantCommand(shooter::setFlywheelSpeedAuto, shooter),
        new InstantCommand(turret::startTracking),
        new ParallelCommandGroup(
          new RunCommand(() -> {intake.deploy(); intake.rollerIntake();}, intake),
          new SequentialCommandGroup(
            new WaitCommand(0.5),
            new InstantCommand(shooter::activate),
            new WaitCommand(5.0),
            new DriveDistanceMeters(drive, 1.5, 0.5)
          )
        )
      );
    }

    public static CommandBase generateTwoBallSimple(Drive drive, Navigation nav, Sensors sensors, Shooter shooter, Turret turret, Intake intake, Hood hood) {
        return new SequentialCommandGroup(
        new SetGlobalPoseToWaypoint(nav, "start_" + getTeamColorName()),
        new TiltCameraDown(sensors),
        new InstantCommand(shooter::setFlywheelSpeedAuto, shooter),
        new InstantCommand(turret::startTracking),
        new ParallelCommandGroup(
          new RunCommand(() -> {intake.deploy(); intake.rollerIntake();}, intake),
          new RunCommand(hood::raiseHood, hood),
          new SequentialCommandGroup(
            new DriveDistanceMeters(drive, 1.5, 0.5),
            new WaitCommand(0.5),
            new InstantCommand(shooter::activate)
          )
        )
      );
    }

      public static CommandBase generateTwoBall(Drive drive, Navigation nav, Sensors sensors, Shooter shooter, Turret turret, Intake intake, Hood hood) {
        return new SequentialCommandGroup(
        new SetGlobalPoseToWaypoint(nav, "start_" + getTeamColorName()),
        new TiltCameraDown(sensors),
        new InstantCommand(shooter::setFlywheelSpeedAuto, shooter),
        new InstantCommand(turret::startTracking),
        new ParallelCommandGroup(
          new RunCommand(() -> {intake.deploy(); intake.rollerIntake();}, intake),
          new RunCommand(hood::raiseHood, hood),
          new SequentialCommandGroup(
            new AutoFollowTrajectory(drive, sensors, RapidReactTrajectories.generateTwoBallTrajectory(), true),
            new WaitCommand(0.5),
            new InstantCommand(shooter::activate),
            new WaitCommand(2.0),
            new InstantCommand(shooter::deactivate)
          )
        )
      );
    }

      public static CommandBase generateTwoBallROS(Drive drive, Navigation nav, Sensors sensors, Shooter shooter, Turret turret, Intake intake, Hood hood, CoprocessorTable rosTable) {
        return new SequentialCommandGroup(
        new SetGlobalPoseToWaypoint(nav, "start_" + getTeamColorName()),
        new TiltCameraDown(sensors),
        new InstantCommand(shooter::setFlywheelSpeedAuto, shooter),
        new InstantCommand(turret::startTracking),
        new ParallelCommandGroup(
          new RunCommand(() -> {intake.deploy(); intake.rollerIntake();}, intake),
          new RunCommand(hood::raiseHood, hood),
          new SequentialCommandGroup(
            new AutoFollowTrajectory(drive, sensors, RapidReactTrajectories.generateTwoBallTrajectory(), true),
            new WaitCommand(0.5),
            new InstantCommand(shooter::activate),
            new WaitCommand(0.5),
            new DriveWithWaypointsPlan(nav, drive, getSingleWaypointPlan(getGameObjectName(), rosTable)),
            new DriveWithWaypointsPlan(nav, drive, getSingleWaypointPlan(getGameObjectName(), rosTable)),
            new DriveWithWaypointsPlan(nav, drive, getSingleWaypointPlan(getGameObjectName(), rosTable))
          )
        )
      );
    }

      public static CommandBase generateThreeBall(Drive drive, Navigation nav, Sensors sensors, Shooter shooter, Turret turret, Intake intake, Hood hood) {
        return new SequentialCommandGroup(
        new SetGlobalPoseToWaypoint(nav, "start_" + getTeamColorName()),
        new TiltCameraDown(sensors),
        new InstantCommand(shooter::setFlywheelSpeedAuto, shooter),
        new InstantCommand(turret::startTracking),
        new ParallelCommandGroup(
          new RunCommand(() -> {intake.deploy(); intake.rollerIntake();}, intake),
          new RunCommand(hood::raiseHood, hood),
          new SequentialCommandGroup(
            new AutoFollowTrajectory(drive, sensors, RapidReactTrajectories.generateTwoBallTrajectory(), true),
            new WaitCommand(0.5),
            new InstantCommand(shooter::activate),
            new WaitCommand(1.0),
            new InstantCommand(shooter::deactivate),
            new AutoFollowTrajectory(drive, sensors, RapidReactTrajectories.generateThreeBallTrajectory(), false),
            new WaitCommand(0.5),
            new InstantCommand(shooter::activate),
            new WaitCommand(1.0),
            new InstantCommand(shooter::deactivate)
          )
        )
      );
    }

      public static CommandBase generateThreeBallDynamic(Drive drive, Navigation nav, Sensors sensors, Shooter shooter, Turret turret, Intake intake, Hood hood) {
        return new SequentialCommandGroup(
        new SetGlobalPoseToWaypoint(nav, "start_" + getTeamColorName()),
        new TiltCameraDown(sensors),
        new InstantCommand(shooter::setFlywheelSpeedAuto, shooter),
        new InstantCommand(turret::startTracking),
        new ParallelCommandGroup(
          new RunCommand(() -> {intake.deploy(); intake.rollerIntake();}, intake),
          new RunCommand(hood::raiseHood, hood),
          new SequentialCommandGroup(
            new AutoFollowTrajectory(drive, sensors, RapidReactTrajectories.generateTwoBallTrajectory(), true),
            new WaitCommand(0.5),
            new InstantCommand(shooter::activate),
            new WaitCommand(1.0),
            new InstantCommand(shooter::deactivate),
            new AutoGoToPose(drive, new Pose2d(Units.feetToMeters(17.0D), Units.feetToMeters(5.5D), Rotation2d.fromDegrees(150.0D))),
            new WaitCommand(0.5),
            new InstantCommand(shooter::activate),
            new WaitCommand(1.0),
            new InstantCommand(shooter::deactivate)
          )
        )
      );
    }

    public static CommandBase generateFourBall(Drive drive, Navigation nav, Sensors sensors, Shooter shooter, Turret turret, Intake intake, Hood hood) {
        return new SequentialCommandGroup(
            new SetGlobalPoseToWaypoint(nav, "start_" + getTeamColorName()),
            new TiltCameraDown(sensors),
            new InstantCommand(shooter::setFlywheelSpeedAuto, shooter),
            new InstantCommand(turret::startTracking),
            new ParallelCommandGroup(
                new RunCommand(() -> {intake.deploy(); intake.rollerIntake();}, intake),
                new RunCommand(hood::raiseHood, hood),
                new SequentialCommandGroup(
                    new AutoFollowTrajectory(drive, sensors, RapidReactTrajectories.generateTwoBallTrajectory(), true),
                    new WaitCommand(0.5),
                    new InstantCommand(shooter::activate),
                    new WaitCommand(1.0),
                    new InstantCommand(shooter::deactivate),
                    new AutoGoToPose(drive, new Pose2d(Units.feetToMeters(new DoublePreferenceConstant("Auto 4 X", 5.5).getValue()), 
                        Units.feetToMeters(new DoublePreferenceConstant("Auto 4 Y", 5.5).getValue()), 
                        Rotation2d.fromDegrees(new DoublePreferenceConstant("Auto 4 Rotation", -140.0).getValue()))),
                    new WaitCommand(new DoublePreferenceConstant("Auto 4 Delay", 3.0).getValue()),
                    new InstantCommand(shooter::activate),
                    new WaitCommand(2.0),
                    new InstantCommand(shooter::deactivate)
                )
            )
        );
    }

      public static CommandBase generateFourBallNoStop(Drive drive, Navigation nav, Sensors sensors, Shooter shooter, Turret turret, Intake intake, Hood hood) {
        return new SequentialCommandGroup(
        new SetGlobalPoseToWaypoint(nav, "start_" + getTeamColorName()),
        new TiltCameraDown(sensors),
        new InstantCommand(shooter::setFlywheelSpeedAuto, shooter),
        new InstantCommand(turret::startTracking),
        new ParallelCommandGroup(
          new RunCommand(() -> {intake.deploy(); intake.rollerIntake();}, intake),
          new RunCommand(hood::raiseHood, hood),
          new AutoFollowTrajectory(drive, sensors, RapidReactTrajectories.generateFourBallNoStopTrajectory(), true),
          new SequentialCommandGroup(
            new InstantCommand(() -> {sensors.limelight.setMotionOffset(new DoublePreferenceConstant("Auto Motion Offset", 0.0).getValue());}), 
            new WaitCommand(2.25),
            new InstantCommand(shooter::activate),
            new WaitCommand(1.0),
            new InstantCommand(shooter::deactivate),
            new InstantCommand(() -> {sensors.limelight.setMotionOffset(0.0);}), 
            new WaitCommand(3.0),
            new InstantCommand(shooter::activate),
            new WaitCommand(2.0),
            new InstantCommand(shooter::deactivate)
          )
        )
      );
    }

    private static WaypointsPlan getSingleWaypointPlan(String waypointName, CoprocessorTable ros_interface) {
      WaypointsPlan autoPlan = new WaypointsPlan(ros_interface);
      autoPlan.addWaypoint(new Waypoint(waypointName));
      return autoPlan;
    }

    private static String getGameObjectName() {
      return "cargo_" + getTeamColorName();
    }
  
    private static  String getTeamColorName() {
      if (DriverStation.getAlliance() == Alliance.Red) {
        return "red";
      }
      else {
        return "blue";
      }
    }
  

}
