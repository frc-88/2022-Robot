// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Sensors;
import frc.robot.util.tunnel.ROSInterface;
import frc.robot.util.tunnel.TunnelServer;
import frc.robot.util.TJController;
import frc.robot.util.drive.DriveUtils;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Sensors m_sensors = new Sensors();
  private final Drive m_drive = new Drive(m_sensors);

  private CommandBase m_teleopDrive;
  private final CommandBase m_autoCommand = new WaitCommand(15.0);

  private final TJController m_driverController = new TJController(0);

  // ROS tunnel interfaces
  private TunnelServer m_tunnel;
  private ROSInterface m_ros_interface;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_ros_interface = new ROSInterface(m_drive);
    m_tunnel = new TunnelServer(m_ros_interface, 5800, 15);

    configureDriverController();
    configureDefaultCommands();
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
    BooleanSupplier arcadeDriveShiftSupplier = () -> !arcadeDriveForceLowGear.getAsBoolean()
        && m_drive.autoshift(arcadeDriveSpeedSupplier.getAsDouble());
    DoubleSupplier arcadeDriveMaxSpeedSupplier = () -> arcadeDriveForceLowGear.getAsBoolean() ? Constants.MAX_SPEED_LOW
        : Constants.MAX_SPEED_HIGH;
    m_teleopDrive = new ArcadeDrive(m_drive, arcadeDriveSpeedSupplier, arcadeDriveTurnSupplier,
        arcadeDriveShiftSupplier, arcadeDriveMaxSpeedSupplier);
    SmartDashboard.putData("Teleop Drive", m_teleopDrive);
  }

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(m_teleopDrive);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
