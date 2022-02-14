// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.AutoFollowTrajectory;
import frc.robot.commands.drive.TankDrive;
import frc.robot.commands.feeder.FeederAcceptCargo;
import frc.robot.commands.turret.TurretTrack;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Sensors;
import frc.robot.util.tunnel.ThisRobotInterface;
import frc.robot.util.tunnel.TunnelServer;
import frc.robot.subsystems.Turret;
import frc.robot.util.RapidReactTrajectories;
import frc.robot.commands.climber.ClimberMotionMagicJoystick;
import frc.robot.commands.climber.ClimberTestMotionMagic;
import frc.robot.commands.climber.ManualModeClimber;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.util.TJController;
import frc.robot.util.drive.DriveUtils;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class RobotContainer {
  // Subsystems
  private final Sensors m_sensors = new Sensors();
  private final Drive m_drive = new Drive(m_sensors);
  private final Turret m_turret = new Turret();
  private final Feeder m_centralizer = new Feeder(Constants.HOPPER_CENTRALIZER_MOTOR_ID, Constants.HOPPER_CENTRALIZER_BEAMBREAK, new DoublePreferenceConstant("Centralizer:Speed", Constants.HOPPER_CENTRALIZER_SPEED_DFT));
  private final Feeder m_chamber = new Feeder(Constants.HOPPER_CHAMBER_MOTOR_ID, Constants.HOPPER_CHAMBER_BEAMBREAK, new DoublePreferenceConstant("Chamber:Speed",Constants.HOPPER_CHAMBER_SPEED_DFT));
  private final Climber m_climber = new Climber();

  // Commands
  private CommandBase m_arcadeDrive;

  private CommandBase m_calibrateClimber;
  private CommandBase m_manualModeClimber;
  private CommandBase m_climberTestMotionMagic;
  private CommandBase m_climberMotionMagicJoystick;

  private final CommandBase m_autoCommand = new WaitCommand(15.0);

  // Controllers
  private final TJController m_driverController = new TJController(0);
  private final TJController m_testController = new TJController(2);

  // ROS tunnel interfaces
  private TunnelServer m_tunnel;
  private ThisRobotInterface m_ros_interface;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_ros_interface = new ThisRobotInterface(m_drive, m_climber.outerLeftArm, m_climber.outerRightArm, m_climber.innerLeftArm, m_climber.innerRightArm, m_turret);
    m_tunnel = new TunnelServer(m_ros_interface, 5800, 15);
    m_calibrateClimber = new RunCommand(m_climber::calibrate, m_climber).withInterrupt(m_climber::isCalibrated).beforeStarting(m_climber::resetCalibration).withName("calibrateClimber");
    m_manualModeClimber = new ManualModeClimber(m_climber, m_testController);
    m_climberTestMotionMagic = new ClimberTestMotionMagic(m_climber);
    m_climberMotionMagicJoystick = new ClimberMotionMagicJoystick(m_climber, m_testController);

    configureDriverController();
    configureDefaultCommands();
    configureDashboardCommands();
  }

  private void configureDriverController() {
    BooleanSupplier arcadeDriveForceLowGear = () -> m_driverController.getRightTrigger() > 0.5;
    DoubleSupplier arcadeDriveSpeedSupplier = DriveUtils.deadbandExponential(m_driverController::getLeftStickY,
        Constants.DRIVE_SPEED_EXP, Constants.DRIVE_JOYSTICK_DEADBAND);
    DoubleSupplier arcadeDriveCheesyDriveMinTurn = () -> arcadeDriveForceLowGear.getAsBoolean()
        ? Constants.CHEESY_DRIVE_FORCE_LOW_MIN_TURN
        : Constants.CHEESY_DRIVE_MIN_TURN;
    DoubleSupplier arcadeDriveCheesyDriveMaxTurn = () -> arcadeDriveForceLowGear.getAsBoolean()
        ? Constants.CHEESY_DRIVE_FORCE_LOW_MAX_TURN
        : Constants.CHEESY_DRIVE_MAX_TURN;
    DoubleSupplier arcadeDriveTurnSupplier = DriveUtils.cheesyTurn(arcadeDriveSpeedSupplier,
        DriveUtils.deadbandExponential(m_driverController::getRightStickX, Constants.DRIVE_SPEED_EXP,
            Constants.DRIVE_JOYSTICK_DEADBAND),
        arcadeDriveCheesyDriveMinTurn.getAsDouble(), arcadeDriveCheesyDriveMaxTurn.getAsDouble());
    Runnable arcadeDriveShiftSupplier = () -> {
      if (arcadeDriveForceLowGear.getAsBoolean()) {
        m_drive.shiftToLow();
      } else {
        m_drive.autoshift(arcadeDriveSpeedSupplier.getAsDouble());
      }
    };
    DoubleSupplier arcadeDriveMaxSpeedSupplier = () -> arcadeDriveForceLowGear.getAsBoolean() ? Constants.MAX_SPEED_LOW
        : Constants.MAX_SPEED_HIGH;

    m_arcadeDrive = new ArcadeDrive(m_drive, arcadeDriveSpeedSupplier, arcadeDriveTurnSupplier,
        arcadeDriveShiftSupplier, arcadeDriveMaxSpeedSupplier);

    CommandBase tankDrive = new TankDrive(m_drive, m_driverController::getLeftStickY, m_driverController::getRightStickY);

    SmartDashboard.putData("Arcade Drive", m_arcadeDrive);
    SmartDashboard.putData("Tank Drive", tankDrive);
  }

  private void configureDashboardCommands() {
    // Centralizer and Chamber commmands
    SmartDashboard.putData("Centralizer:AcceptCargo", new FeederAcceptCargo(m_centralizer));
    SmartDashboard.putData("Centralizer:Run", new InstantCommand(m_centralizer::run, m_centralizer));
    SmartDashboard.putData("Centralizer:Reverse", new InstantCommand(m_centralizer::reverse, m_centralizer));
    SmartDashboard.putData("Centralizer:Stop", new InstantCommand(m_centralizer::stop, m_centralizer));
    SmartDashboard.putData("Chamber:AcceptCargo", new FeederAcceptCargo(m_chamber));
    SmartDashboard.putData("Chamber:Run", new InstantCommand(m_chamber::run, m_chamber));
    SmartDashboard.putData("Chamber:Reverse", new InstantCommand(m_chamber::reverse, m_chamber));
    SmartDashboard.putData("Chamber:Stop", new InstantCommand(m_chamber::stop, m_chamber));
    
    SmartDashboard.putData(m_calibrateClimber);
    SmartDashboard.putData(m_manualModeClimber);
    SmartDashboard.putData(m_climberTestMotionMagic);
    SmartDashboard.putData(m_climberMotionMagicJoystick);

    // Trajectory testing commands
    SmartDashboard.putData("Ten Feet Forward", new AutoFollowTrajectory(m_drive, m_sensors, RapidReactTrajectories.generateTestTrajectory()));
    SmartDashboard.putData("Barrel Run", new AutoFollowTrajectory(m_drive, m_sensors, RapidReactTrajectories.generateBarrelRunTrajectory()));
    SmartDashboard.putData("Barrel Run 2", new AutoFollowTrajectory(m_drive, m_sensors, RapidReactTrajectories.generateBarrelRun2Trajectory()));

    // Turret test commands
    SmartDashboard.putData("Turret Start Tracking", new InstantCommand(m_turret::startTracking));
    SmartDashboard.putData("Turret Stop Tracking", new InstantCommand(m_turret::stopTracking));
  }

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(m_arcadeDrive);
    m_turret.setDefaultCommand(new TurretTrack(m_turret, m_sensors.limelight));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}
