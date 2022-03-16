// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
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
    private static CommandBase generatePrepareCmd(Sensors sensors, Shooter shooter, Turret turret, Intake intake, Hood hood, boolean hoodUp) {
        return new ParallelCommandGroup(
            new TiltCameraDown(sensors),
            new InstantCommand(turret::startTracking),
            new RunCommand(shooter::setFlywheelSpeedAuto, shooter),
            new RunCommand(() -> {intake.deploy(); intake.rollerIntake();}, intake),
            new RunCommand(hoodUp ? hood::raiseHood : hood::lowerHood, hood)
        );
    }

    private static CommandBase generateShootCmd(Shooter shooter) {
        return new SequentialCommandGroup(
            new InstantCommand(shooter::activate),
            new WaitCommand(1.0),
            new InstantCommand(shooter::deactivate)
        );
    }

    public static CommandBase generateOneBall(Drive drive, Navigation nav, Sensors sensors, Shooter shooter, Turret turret, Intake intake, Hood hood) {
        return new SequentialCommandGroup(
            new SetGlobalPoseToWaypoint(nav, "start_" + getTeamColorName()),
            new ParallelCommandGroup(
                generatePrepareCmd(sensors, shooter, turret, intake, hood, false),
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    generateShootCmd(shooter),
                    new WaitCommand(3.0),
                    new DriveDistanceMeters(drive, 1.5, 0.5)
                )
            )
        );
    }

    public static CommandBase generateTwoBallSimple(Drive drive, Navigation nav, Sensors sensors, Shooter shooter, Turret turret, Intake intake, Hood hood) {
        return new SequentialCommandGroup(
            new SetGlobalPoseToWaypoint(nav, "start_" + getTeamColorName()),
            new ParallelCommandGroup(
                generatePrepareCmd(sensors, shooter, turret, intake, hood, false),
                new SequentialCommandGroup(
                    new DriveDistanceMeters(drive, 1.5, 0.5),
                    new WaitCommand(0.5),
                    generateShootCmd(shooter)
                )
            )
        );
    }

    public static CommandBase generateTwoBall(Drive drive, Navigation nav, Sensors sensors, Shooter shooter, Turret turret, Intake intake, Hood hood) {
        return new SequentialCommandGroup(
            new SetGlobalPoseToWaypoint(nav, "start_" + getTeamColorName()),
            new ParallelCommandGroup(
                generatePrepareCmd(sensors, shooter, turret, intake, hood, false),
                new SequentialCommandGroup(
                    new AutoFollowTrajectory(drive, RapidReactTrajectories.generateTwoBallTrajectory(), true),
                    new WaitCommand(0.5),
                    generateShootCmd(shooter)
                )
            )
        );
    }   

      public static CommandBase generateTwoBallROS(Drive drive, Navigation nav, Sensors sensors, Shooter shooter, Turret turret, Intake intake, Hood hood, CoprocessorTable rosTable) {
        return new SequentialCommandGroup(
            new SetGlobalPoseToWaypoint(nav, "start_" + getTeamColorName()),
            new ParallelCommandGroup(
                generatePrepareCmd(sensors, shooter, turret, intake, hood, false),
                new SequentialCommandGroup(
                    new AutoFollowTrajectory(drive, RapidReactTrajectories.generateTwoBallTrajectory(), true),
                    new WaitCommand(0.5),
                    new InstantCommand(shooter::activate),
                    new WaitCommand(0.5),
                    new DriveWithWaypointsPlan(nav, drive, getSingleWaypointPlan(getGameObjectName(), rosTable)),
                    new WaitCommand(0.5),
                    new InstantCommand(shooter::activate),
                    new WaitCommand(0.5)
                )
            )
        );
    }

      public static CommandBase generateThreeBall(Drive drive, Navigation nav, Sensors sensors, Shooter shooter, Turret turret, Intake intake, Hood hood) {
        return new SequentialCommandGroup(
            new SetGlobalPoseToWaypoint(nav, "start_" + getTeamColorName()),
            new ParallelCommandGroup(
                generatePrepareCmd(sensors, shooter, turret, intake, hood, false),
                new SequentialCommandGroup(
                    new AutoFollowTrajectory(drive, RapidReactTrajectories.generateTwoBallTrajectory(), true),
                    new WaitCommand(0.5),
                    generateShootCmd(shooter),
                    new AutoFollowTrajectory(drive, RapidReactTrajectories.generateThreeBallTrajectory(), false),
                    new WaitCommand(0.5),
                    generateShootCmd(shooter)
                )
            )
        );
    }

      public static CommandBase generateThreeBallDynamic(Drive drive, Navigation nav, Sensors sensors, Shooter shooter, Turret turret, Intake intake, Hood hood) {
        return new SequentialCommandGroup(
            new SetGlobalPoseToWaypoint(nav, "start_" + getTeamColorName()),
            new ParallelCommandGroup(
                generatePrepareCmd(sensors, shooter, turret, intake, hood, false),
                new SequentialCommandGroup(
                    new AutoFollowTrajectory(drive, RapidReactTrajectories.generateTwoBallTrajectory(), true),
                    new WaitCommand(0.5),
                    generateShootCmd(shooter),
                    new AutoGoToPose(drive, new Pose2d(Units.feetToMeters(17.0D), Units.feetToMeters(5.5D), Rotation2d.fromDegrees(150.0D))),
                    new WaitCommand(0.5),
                    generateShootCmd(shooter)
                )
            )
        );
    }

    public static CommandBase generateFourBall(Drive drive, Navigation nav, Sensors sensors, Shooter shooter, Turret turret, Intake intake, Hood hood) {
        return new SequentialCommandGroup(
            new SetGlobalPoseToWaypoint(nav, "start_" + getTeamColorName()),
            new ParallelCommandGroup(
                generatePrepareCmd(sensors, shooter, turret, intake, hood, false),
                new SequentialCommandGroup(
                    new AutoFollowTrajectory(drive, RapidReactTrajectories.generateTwoBallTrajectory(), true),
                    new WaitCommand(0.5),
                    generateShootCmd(shooter),
                    new AutoGoToPose(drive, new Pose2d(Units.feetToMeters(new DoublePreferenceConstant("Auto 4 X", 5.5).getValue()), 
                        Units.feetToMeters(new DoublePreferenceConstant("Auto 4 Y", 5.5).getValue()), 
                        Rotation2d.fromDegrees(new DoublePreferenceConstant("Auto 4 Rotation", -133.75).getValue()))),
                    new WaitCommand(new DoublePreferenceConstant("Auto 4 Delay", 3.0).getValue()),
                    generateShootCmd(shooter)
                )
            )
        );
    }

    public static CommandBase generateFourBallNoStop(Drive drive, Navigation nav, Sensors sensors, Shooter shooter, Turret turret, Intake intake, Hood hood) {
        Trajectory trajectory = RapidReactTrajectories.generateFourBallNoStopTrajectory();
        return new SequentialCommandGroup(
            new SetGlobalPoseToWaypoint(nav, "start_" + getTeamColorName()),
            new ParallelCommandGroup(
                generatePrepareCmd(sensors, shooter, turret, intake, hood, false),
                new AutoFollowTrajectory(drive, trajectory, true),
                new SequentialCommandGroup(
                    new InstantCommand(() -> {sensors.limelight.setMotionOffset(new DoublePreferenceConstant("Auto Motion Offset", 0.0).getValue());}), 
                    new WaitCommand(new DoublePreferenceConstant("Auto 4X Shot Delay", 2.25).getValue()),
                    generateShootCmd(shooter),
                    new InstantCommand(() -> {sensors.limelight.setMotionOffset(0.0);}), 
                    new WaitCommand(trajectory.getTotalTimeSeconds()),
                    generateShootCmd(shooter)
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
