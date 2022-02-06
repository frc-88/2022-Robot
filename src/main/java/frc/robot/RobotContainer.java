// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.AutoFollowTrajectory;
import frc.robot.commands.drive.TankDrive;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Sensors;
import frc.robot.util.RapidReactTrajectories;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.util.TJController;
import frc.robot.util.drive.DriveUtils;

public class RobotContainer {
  private Orchestra m_orchestra = new Orchestra();
  // Subsystems
  private final Sensors m_sensors = new Sensors();
  private final Drive m_drive = new Drive(m_sensors);

  // Commands
  private CommandBase m_arcadeDrive;
  private final CommandBase m_autoCommand = new WaitCommand(15.0);

  // Controllers
  private final TJController m_driverController = new TJController(0);

  public RobotContainer() {
    m_orchestra.loadMusic("Moveslikejagger.chrp");
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
    BooleanSupplier arcadeDriveShiftSupplier = () -> !arcadeDriveForceLowGear.getAsBoolean()
        && m_drive.autoshift(arcadeDriveSpeedSupplier.getAsDouble());
    DoubleSupplier arcadeDriveMaxSpeedSupplier = () -> arcadeDriveForceLowGear.getAsBoolean() ? Constants.MAX_SPEED_LOW
        : Constants.MAX_SPEED_HIGH;

    m_arcadeDrive = new ArcadeDrive(m_drive, arcadeDriveSpeedSupplier, arcadeDriveTurnSupplier,
        arcadeDriveShiftSupplier, arcadeDriveMaxSpeedSupplier);

    CommandBase tankDrive = new TankDrive(m_drive, m_driverController::getLeftStickY, m_driverController::getRightStickY);

    SmartDashboard.putData("Arcade Drive", m_arcadeDrive);
    SmartDashboard.putData("Tank Drive", tankDrive);
  }

  private void configureDashboardCommands() {
    SmartDashboard.putData("Ten Feet Forward", new AutoFollowTrajectory(m_drive, m_sensors, RapidReactTrajectories.generateTestTrajectory()));
    SmartDashboard.putData("Barrel Run", new AutoFollowTrajectory(m_drive, m_sensors, RapidReactTrajectories.generateBarrelRunTrajectory()));
    SmartDashboard.putData("Barrel Run 2", new AutoFollowTrajectory(m_drive, m_sensors, RapidReactTrajectories.generateBarrelRun2Trajectory()));

    SmartDashboard.putData("Music", new InstantCommand(m_orchestra::play));
  }

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(m_arcadeDrive);
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
