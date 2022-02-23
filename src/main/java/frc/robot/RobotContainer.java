// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.AutoFollowTrajectory;
import frc.robot.commands.feeder.FeederAcceptCargo;
import frc.robot.commands.feeder.FeederCargolizer;
import frc.robot.commands.turret.TurretCalibrate;
import frc.robot.commands.turret.TurretMotionMagicJoystick;
import frc.robot.commands.turret.TurretRawJoystick;
import frc.robot.commands.turret.TurretTrack;
import frc.robot.commands.feeder.FeederCargolizer;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Shooter;
import frc.robot.util.tunnel.ThisRobotInterface;
import frc.robot.util.tunnel.TunnelServer;
import frc.robot.subsystems.Turret;
import frc.robot.util.RapidReactTrajectories;
import frc.robot.util.controllers.ButtonBox;
import frc.robot.util.controllers.DriverController;
import frc.robot.util.controllers.FrskyDriverController;
import frc.robot.util.controllers.XboxController;
import frc.robot.commands.LimelightToggle;
import frc.robot.commands.climber.ClimberMotionMagicJoystick;
import frc.robot.commands.climber.ClimberTestMotionMagic;
import frc.robot.commands.climber.ManualModeClimber;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class RobotContainer {
  /////////////////////////////////////////////////////////////////////////////
  //                              SUBSYSTEMS                                 //
  /////////////////////////////////////////////////////////////////////////////
  private final Sensors m_sensors = new Sensors();
  private final Drive m_drive = new Drive(m_sensors);
  private final Shooter m_shooter = new Shooter(m_sensors.limelight);
  private final Intake m_intake = new Intake();
  private final Turret m_turret = new Turret();
  private final Feeder m_centralizer = new Feeder("Centralizer",Constants.FEEDER_CENTRALIZER_MOTOR_ID, Constants.FEEDER_CENTRALIZER_BEAMBREAK, new DoublePreferenceConstant("Centralizer:Speed", Constants.FEEDER_CENTRALIZER_SPEED_DFT));
  private final Feeder m_chamber = new Feeder("Chamber",Constants.FEEDER_CHAMBER_MOTOR_ID, Constants.FEEDER_CHAMBER_BEAMBREAK, new DoublePreferenceConstant("Chamber:Speed",Constants.FEEDER_CHAMBER_SPEED_DFT));
  private final Climber m_climber = new Climber();

  

  /////////////////////////////////////////////////////////////////////////////
  //                              CONTROLLERS                                //
  /////////////////////////////////////////////////////////////////////////////
  private final DriverController m_driverController = new FrskyDriverController(Constants.DRIVER_CONTROLLER_ID);
  private final ButtonBox m_buttonBox = new ButtonBox(Constants.BUTTON_BOX_ID);
  private final XboxController m_testController = new XboxController(Constants.TEST_CONTROLLER_ID);


  
  /////////////////////////////////////////////////////////////////////////////
  //                                 ROS                                     //
  /////////////////////////////////////////////////////////////////////////////
  private ThisRobotInterface m_ros_interface = new ThisRobotInterface(
    m_drive,
    m_climber.outerLeftArm, m_climber.outerRightArm, m_climber.innerLeftArm, m_climber.innerRightArm,
    m_intake,
    m_turret);
  private TunnelServer m_tunnel = new TunnelServer(m_ros_interface, 5800, 15);


  /////////////////////////////////////////////////////////////////////////////
  //                              PREFERENCES                                //
  /////////////////////////////////////////////////////////////////////////////
  private DoublePreferenceConstant p_shooterTestOutput = new DoublePreferenceConstant("Shooter Test Output", 0.0);
  private DoublePreferenceConstant p_shooterTestVelocity = new DoublePreferenceConstant("Shooter Test Speed", 0.0);


  /////////////////////////////////////////////////////////////////////////////
  //                               COMMANDS                                  //
  /////////////////////////////////////////////////////////////////////////////
  

  /////////////////////////////////////
  //             DRIVE               //
  /////////////////////////////////////

  private CommandBase m_arcadeDrive =
      new ArcadeDrive(m_drive, m_driverController::getThrottle, m_driverController::getTurn,
          () -> {
            if (m_driverController.getForceLowGear()) {
              m_drive.shiftToLow();
            } else {
              m_drive.autoshift(m_driverController.getThrottle());
            }
          }, 
          () -> m_driverController.getForceLowGear() ? Constants.MAX_SPEED_LOW : Constants.MAX_SPEED_HIGH);


  /////////////////////////////////////
  //          BALL HANDLING          //
  /////////////////////////////////////

  private CommandBase m_ingestCargo = new ParallelCommandGroup(new RunCommand(() -> {
        m_intake.deploy();
        m_intake.rollerIntake();
      }, m_intake),
      new FeederAcceptCargo(m_centralizer));

  private CommandBase m_outgestCargo = new ParallelCommandGroup(new RunCommand(() -> {
        m_intake.deploy();
        m_intake.rollerOutgest();
      }, m_intake),
      new InstantCommand(m_centralizer::reverse, m_centralizer));

  private CommandBase m_stowIntake = new RunCommand(() -> {
        m_intake.stow();
        m_intake.rollerStop();
      }, m_intake);

  private CommandBase m_stowIntakeStopCentralizer = new ParallelCommandGroup(new RunCommand(() -> {
    m_intake.stow();
    m_intake.rollerStop();
  }, m_intake),
      new InstantCommand(m_centralizer::stop, m_centralizer));

  private CommandBase m_centralizerCargolizer = new FeederCargolizer(m_centralizer, m_intake, m_chamber);
  private CommandBase m_chamberCargolizer = new FeederCargolizer(m_chamber, m_centralizer, m_shooter);

  /////////////////////////////////////
  //            SHOOTING             //
  /////////////////////////////////////

  private CommandBase m_startFlywheelRaw = new InstantCommand(() -> {m_shooter.setFlywheelRaw(p_shooterTestOutput.getValue());}, m_shooter);
  private CommandBase m_stopFlywheelRaw = new InstantCommand(() -> {m_shooter.setFlywheelRaw(0.0);}, m_shooter);
  private CommandBase m_startFlywheel = new InstantCommand(() -> {m_shooter.setFlywheelSpeed(p_shooterTestVelocity.getValue());}, m_shooter);
  private CommandBase m_stopFlywheel = new InstantCommand(() -> {m_shooter.setFlywheelSpeed(0.0);}, m_shooter);

  private CommandBase m_shoot = new SequentialCommandGroup(
    new InstantCommand(m_centralizer::run, m_centralizer),
    new InstantCommand(m_chamber::run, m_chamber),
    new WaitCommand(0.5),
    new InstantCommand(m_centralizer::stop, m_centralizer),
    new InstantCommand(m_chamber::stop, m_chamber));

  private CommandBase m_turretTrackingOn = new InstantCommand(m_turret::startTracking);

  private CommandBase m_stowShooter = new InstantCommand(m_turret::stopTracking);

  private CommandBase m_hoodUp = new RunCommand(m_shooter::raiseHood, m_shooter);

  private CommandBase m_hoodDown = new RunCommand(m_shooter::lowerHood, m_shooter);

  /////////////////////////////////////
  //             CLIMBER             //
  /////////////////////////////////////

  private CommandBase m_calibrateClimber = 
      new RunCommand(m_climber::calibrate, m_climber)
          .withInterrupt(m_climber::isCalibrated)
          .beforeStarting(m_climber::resetCalibration)
          .withName("calibrateClimber");

  private CommandBase m_stowClimber = new WaitCommand(1);

  private CommandBase m_manualModeClimber = new ManualModeClimber(m_climber, m_testController);
  private CommandBase m_climberTestMotionMagic = new ClimberTestMotionMagic(m_climber);;
  private CommandBase m_climberMotionMagicJoystick = new ClimberMotionMagicJoystick(m_climber, m_testController);

  /////////////////////////////////////
  //              AUTO               //
  /////////////////////////////////////

  private CommandBase m_autoCommand = new WaitCommand(15.0);

  /////////////////////////////////////////////////////////////////////////////
  //                                 SETUP                                   //
  /////////////////////////////////////////////////////////////////////////////

  public RobotContainer() {
    configureButtonBox();
    configureDefaultCommands();
    configureDashboardCommands();
  }

  public void disabledPeriodic() {
    if (m_buttonBox.isShootButtonPressed()) {
      m_autoCommand = new ParallelCommandGroup(
        new InstantCommand(() -> {m_shooter.setFlywheelRaw(p_shooterTestOutput.getValue());}, m_shooter),
        new SequentialCommandGroup(
          new WaitCommand(6),
          new InstantCommand(m_centralizer::run, m_centralizer),
          new InstantCommand(m_chamber::run, m_chamber),
          new WaitCommand(0.5),
          new InstantCommand(m_centralizer::stop, m_centralizer),
          new InstantCommand(m_chamber::stop, m_chamber)
        )
      );
    }
  }

  private void configureButtonBox() {
    m_buttonBox.intakeButton.whileHeld(m_ingestCargo);
    m_buttonBox.outgestButton.whileHeld(m_outgestCargo);
    m_buttonBox.shootButton.whenPressed(m_shoot);
    m_buttonBox.shooterButton.whenPressed(m_startFlywheelRaw);
    m_buttonBox.shooterButton.whenReleased(m_stopFlywheelRaw);
    //m_buttonBox.hoodSwitch.whenPressed(m_hoodUp);
    //m_buttonBox.hoodSwitch.whenReleased(m_hoodDown);
  }

  private void configureDashboardCommands() {
    // Drive testing commands
    SmartDashboard.putData("Drive forwards", new ArcadeDrive(m_drive, () -> 5, () -> 0, () -> m_drive.shiftToLow(), () -> Constants.MAX_SPEED_LOW));

    // Trajectory testing commands
    // SmartDashboard.putData("Ten Feet Forward", new AutoFollowTrajectory(m_drive, m_sensors, RapidReactTrajectories.generateTestTrajectory()));
    // SmartDashboard.putData("Barrel Run", new AutoFollowTrajectory(m_drive, m_sensors, RapidReactTrajectories.generateBarrelRunTrajectory()));
    // SmartDashboard.putData("Barrel Run 2", new AutoFollowTrajectory(m_drive, m_sensors, RapidReactTrajectories.generateBarrelRun2Trajectory()));

    // Intake testing commands
    SmartDashboard.putData("Intake:Ingest", m_ingestCargo);
    SmartDashboard.putData("Intake:Stow", m_stowIntakeStopCentralizer);

    // Centralizer and Chamber commmands
    SmartDashboard.putData("Centralizer:AcceptCargo", new FeederAcceptCargo(m_centralizer));
    SmartDashboard.putData("Centralizer:Run", new InstantCommand(m_centralizer::run, m_centralizer));
    SmartDashboard.putData("Centralizer:Reverse", new InstantCommand(m_centralizer::reverse, m_centralizer));
    SmartDashboard.putData("Centralizer:Stop", new InstantCommand(m_centralizer::stop, m_centralizer));
    SmartDashboard.putData("Centralizer:Cargolizer", m_centralizerCargolizer);
    
    SmartDashboard.putData("Chamber:AcceptCargo", new FeederAcceptCargo(m_chamber));
    SmartDashboard.putData("Chamber:Run", new InstantCommand(m_chamber::run, m_chamber));
    SmartDashboard.putData("Chamber:Reverse", new InstantCommand(m_chamber::reverse, m_chamber));
    SmartDashboard.putData("Chamber:Stop", new InstantCommand(m_chamber::stop, m_chamber));
    SmartDashboard.putData("Chamber:Cargolizer", m_chamberCargolizer);

    // Turret test commands
    SmartDashboard.putData("Turret Raw Control",new TurretRawJoystick(m_turret, m_testController));
    SmartDashboard.putData("Turret Motion Magic Control",new TurretMotionMagicJoystick(m_turret, m_testController));
    SmartDashboard.putData("Turret Go To 45", new InstantCommand(() -> {m_turret.goToFacing(45);}, m_turret));
    SmartDashboard.putData("Turret Go To 0", new InstantCommand(() -> {m_turret.goToFacing(0);}, m_turret));
    
    SmartDashboard.putData("Turret Track", new TurretTrack(m_turret, m_sensors.limelight));
    SmartDashboard.putData("Turret Activate Tracking", m_turretTrackingOn);
    SmartDashboard.putData("Turret Deactivate Tracking", new InstantCommand(m_turret::stopTracking));
    SmartDashboard.putData("Turret Calibrate", new TurretCalibrate(m_turret));
    SmartDashboard.putData("Turret Sync", new InstantCommand(m_turret::sync, m_turret));

    // Shooter testing commands
    SmartDashboard.putData("Shooter:Flywheel:RunRaw", m_startFlywheelRaw);
    SmartDashboard.putData("Shooter:Flywheel:RunSpeed", m_startFlywheel);
    SmartDashboard.putData("Shooter:Flywheel:StopRaw", m_stopFlywheelRaw);
    SmartDashboard.putData("Shooter:Flywheel:StopSpeed", m_stopFlywheel);
    SmartDashboard.putData("Shooter:Hood:Raise", new RunCommand(m_shooter::raiseHood, m_sensors));
    SmartDashboard.putData("Shooter:Hood:Lower", new RunCommand(m_shooter::lowerHood, m_sensors));
    SmartDashboard.putData("Shooter:Activate", new InstantCommand(m_shooter::activate, m_shooter));
    SmartDashboard.putData("Shooter:Deactivate", new InstantCommand(m_shooter::deactivate, m_shooter));

    // Limelight
    SmartDashboard.putData("Limelight On", new LimelightToggle(m_sensors.limelight, true));
    SmartDashboard.putData("Limelight Off", new LimelightToggle(m_sensors.limelight, false));

    // Climber Commands
    SmartDashboard.putData(m_calibrateClimber);
    SmartDashboard.putData(m_manualModeClimber);
    SmartDashboard.putData(m_climberTestMotionMagic);
    SmartDashboard.putData(m_climberMotionMagicJoystick);
  }

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(m_arcadeDrive);
    m_intake.setDefaultCommand(m_stowIntake);


    //m_turret.setDefaultCommand(new TurretTrack(m_turret, m_sensors.limelight));


    // m_climber.setDefaultCommand( 
    //   new SequentialCommandGroup(
    //     new RunCommand(m_climber::calibrate, m_climber)
    //       .withInterrupt(m_climber::isCalibrated)
    //       .beforeStarting(m_climber::resetCalibration)
    //       .withName("calibrateClimber"),
    //     new ClimberMotionMagicJoystick(m_climber, m_testController)
    //   ));
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
