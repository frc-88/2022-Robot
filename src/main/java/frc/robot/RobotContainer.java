// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import javax.management.InstanceAlreadyExistsException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.AutoFollowTrajectory;
import frc.robot.commands.drive.DriveDistanceMeters;
import frc.robot.commands.drive.TankDrive;
import frc.robot.commands.feeder.FeederAcceptCargo;
import frc.robot.commands.feeder.FeederCargolizer;
import frc.robot.commands.feeder.FeederOutgestCargo;
import frc.robot.commands.feeder.FeederPassthru;
import frc.robot.commands.turret.TurretCalibrate;
import frc.robot.commands.turret.TurretLock;
import frc.robot.commands.turret.TurretMotionMagicJoystick;
import frc.robot.commands.turret.TurretRawJoystick;
import frc.robot.commands.turret.TurretTrack;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.DummySubsytem;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.util.CargoSource;
import frc.robot.util.RapidReactTrajectories;
import frc.robot.util.climber.ClimberConstants;
import frc.robot.util.controllers.ButtonBox;
import frc.robot.util.controllers.DriverController;
import frc.robot.util.controllers.FrskyDriverController;
import frc.robot.util.controllers.XboxController;
import frc.robot.util.controllers.ButtonBox.ClimbBar;
import frc.robot.util.ThisRobotTable;
import frc.robot.commands.LimelightHoodToggle;
import frc.robot.commands.LimelightToggle;
import frc.robot.commands.autos.DriveWithWaypointsPlan;
import frc.robot.commands.cameratilter.TiltCameraDown;
import frc.robot.commands.climber.ClimberMotionMagicJoystick;
import frc.robot.commands.climber.ClimberStateMachineExecutor;
import frc.robot.commands.climber.ClimberTestMotionMagic;
import frc.robot.commands.climber.ManualModeClimber;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.roswaypoints.Waypoint;
import frc.robot.util.roswaypoints.WaypointsPlan;


public class RobotContainer {
  /////////////////////////////////////////////////////////////////////////////
  //                              SUBSYSTEMS                                 //
  /////////////////////////////////////////////////////////////////////////////
  private final Sensors m_sensors = new Sensors();
  private final Drive m_drive = new Drive(m_sensors);
  private final Intake m_intake = new Intake();
  private final Turret m_turret = new Turret();
  private final Feeder m_centralizer = new Feeder("Centralizer",Constants.FEEDER_CENTRALIZER_MOTOR_ID, Constants.FEEDER_CENTRALIZER_BEAMBREAK, new DoublePreferenceConstant("Centralizer:In", -0.3), new DoublePreferenceConstant("Centralizer:Out", -0.3));
  private final Feeder m_chamber = new Feeder("Chamber",Constants.FEEDER_CHAMBER_MOTOR_ID, Constants.FEEDER_CHAMBER_BEAMBREAK, new DoublePreferenceConstant("Chamber:In", 0.2), new DoublePreferenceConstant("Chamber:Out", 0.6));
  private final Shooter m_shooter = new Shooter(new CargoSource[]{m_centralizer, m_chamber}, m_sensors);
  private final Climber m_climber = new Climber(m_sensors::isCoastButtonPressed);
  private final DummySubsytem m_hoodDummy = new DummySubsytem();
  

  /////////////////////////////////////////////////////////////////////////////
  //                              CONTROLLERS                                //
  /////////////////////////////////////////////////////////////////////////////
  private final DriverController m_driverController = new FrskyDriverController(Constants.DRIVER_CONTROLLER_ID);
  private final ButtonBox m_buttonBox = new ButtonBox(Constants.BUTTON_BOX_ID);
  private final XboxController m_testController = new XboxController(Constants.TEST_CONTROLLER_ID);
  private final XboxController m_testController2 = new XboxController(Constants.TEST_CONTROLLER_2_ID);

  
  /////////////////////////////////////////////////////////////////////////////
  //                                 ROS                                     //
  /////////////////////////////////////////////////////////////////////////////
  private final ThisRobotTable m_ros_interface = new ThisRobotTable(m_drive, "10.0.88.44", 5800, 0.01,
    m_climber.outerArm, m_climber.innerArm, m_intake, m_turret, m_sensors
  );
  private final Navigation m_nav = new Navigation(m_ros_interface);


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

  private Supplier<CommandBase> m_ingestCargo = () -> {return new RunCommand(() -> {
        m_intake.deploy();
        m_intake.rollerIntake();
      }, m_intake);
    };

  private CommandBase m_outgestCargo = new ParallelCommandGroup(new RunCommand(() -> {
        m_intake.deploy();
        m_intake.rollerOutgest();
      }, m_intake),
      new FeederOutgestCargo(m_centralizer));

  private CommandBase m_stowIntake = new RunCommand(() -> {
        m_intake.stow();
        m_intake.rollerStop();
      }, m_intake);

  private CommandBase m_stowIntakeTwo = new RunCommand(() -> {
    m_intake.stow();
    m_intake.rollerStop();
  }, m_intake);

  private CommandBase m_stowIntakeAndShooter = new RunCommand(() -> {
    m_intake.stow();
    m_intake.rollerStop();
    m_shooter.setFlywheelSpeed(0.0);
  }, m_intake, m_shooter);

  private Supplier<CommandBase> m_centralizerCargolizer = () -> { return new FeederCargolizer(m_centralizer, m_intake, m_chamber); };
  private Supplier<CommandBase> m_chamberCargolizer = () -> { return new FeederCargolizer(m_chamber, m_centralizer, m_shooter); };

  /////////////////////////////////////
  //            SHOOTING             //
  /////////////////////////////////////

  private CommandBase m_startFlywheelRaw = new InstantCommand(() -> {m_shooter.setFlywheelRaw(p_shooterTestOutput.getValue());}, m_shooter);
  private CommandBase m_stopFlywheelRaw = new InstantCommand(() -> {m_shooter.setFlywheelRaw(0.0);}, m_shooter);
  private Supplier<CommandBase> m_startFlywheel = () -> {return new InstantCommand(() -> {m_shooter.setFlywheelSpeed(p_shooterTestVelocity.getValue());}, m_shooter); };
  private CommandBase m_stopFlywheel = new InstantCommand(() -> {m_shooter.setFlywheelSpeed(-500);}, m_shooter);

  private CommandBase m_shoot = new SequentialCommandGroup(
    new InstantCommand(m_centralizer::run, m_centralizer),
    new InstantCommand(m_chamber::run, m_chamber),
    new WaitCommand(0.5),
    new InstantCommand(m_centralizer::stop, m_centralizer),
    new InstantCommand(m_chamber::stop, m_chamber));

  private CommandBase m_turretTrackingOn = new InstantCommand(m_turret::startTracking);

  private CommandBase m_stowShooter = new InstantCommand(m_turret::stopTracking);

  private CommandBase m_hoodUp = new RunCommand(m_shooter::raiseHood, m_hoodDummy);

  private CommandBase m_hoodDown = new RunCommand(m_shooter::lowerHood, m_hoodDummy);

  /////////////////////////////////////
  //             CLIMBER             //
  /////////////////////////////////////

  private CommandBase m_calibrateClimber = 
      new RunCommand(m_climber::calibrate, m_climber)
          .withInterrupt(m_climber::isCalibrated)
          .beforeStarting(m_climber::resetCalibration)
          .withName("calibrateClimber");

  private CommandBase m_manualModeClimber = new ManualModeClimber(m_climber, m_testController);
  private CommandBase m_climberTestMotionMagic = new ClimberTestMotionMagic(m_climber);
  private CommandBase m_climberMotionMagicJoystick = new ClimberMotionMagicJoystick(m_climber, m_testController);

  /////////////////////////////////////
  //              AUTO               //
  /////////////////////////////////////

  private CommandBase m_autoCommand;
  // private CommandBase m_pursueCargoCommand;
  // private CommandBase m_allowRosCommandVelocities = new AllowRosCommands(m_drive, m_ros_interface);

  /////////////////////////////////////////////////////////////////////////////
  //                                 SETUP                                   //
  /////////////////////////////////////////////////////////////////////////////

  public RobotContainer(Robot robot) {
    setupAutonomousCommand(0);
    configurePeriodics(robot);
    configureButtonBox();
    configureDefaultCommands();
    configureDashboardCommands();
  }

  private void configurePeriodics(Robot robot) {
    robot.addPeriodic(m_ros_interface::update, 1.0 / 60.0, 0.01);
  }

  private CommandBase setupSimpleAuto() { 
    return new SequentialCommandGroup(
      new TiltCameraDown(m_sensors),
      new ParallelCommandGroup(
        m_ingestCargo.get(),
        new SequentialCommandGroup(
          m_startFlywheel.get(),
          new WaitCommand(0.5),
          new InstantCommand(m_shooter::activate),
          new WaitCommand(2.0),
          new InstantCommand(m_shooter::deactivate),
          new ParallelCommandGroup(
            new RunCommand(m_shooter::setFlywheelSpeedAuto, m_shooter),
            new RunCommand(m_shooter::raiseHood, m_hoodDummy),
            new SequentialCommandGroup(
              new ArcadeDrive(m_drive, () -> .15, () -> 0, () -> m_drive.shiftToLow(), () -> Constants.MAX_SPEED_LOW).withTimeout(3),
              new ParallelCommandGroup(
                new ArcadeDrive(m_drive, () -> 0, () -> 0, () -> m_drive.shiftToLow(), () -> Constants.MAX_SPEED_LOW),
                new WaitCommand(1.5)
                // new InstantCommand(m_shooter::activate)
              )
            )
          )
        )
      )
    );
  }
  
  public String getGameObjectName() {
    return "cargo_" + getTeamColorName();
}

  private String getTeamColorName() {
    if (DriverStation.getAlliance() == Alliance.Red) {
      return "red";
    }
    else {
      return "blue";
    }
  }

  private CommandBase setupAutonomousCommand(int autoIndex)
  {
    String team_color = getTeamColorName();

    WaypointsPlan autoPlan = new WaypointsPlan(m_ros_interface);
    autoPlan.addWaypoint(new Waypoint(team_color + "_" + autoIndex + "_point_a"));
    autoPlan.addWaypoint(new Waypoint(team_color + "_" + autoIndex + "_point_b"));
    autoPlan.addWaypoint(new Waypoint(team_color + "_" + autoIndex + "_end"));
    autoPlan.addWaypoint(new Waypoint(getGameObjectName()));

    return new SequentialCommandGroup(
      // new DriveDistanceMeters(m_drive, 0.5, 0.5),
      new DriveWithWaypointsPlan(m_nav, m_drive, autoPlan)
    );
  }
  
  public void disabledPeriodic() {
    if (m_buttonBox.isShootButtonPressed()) {
      // m_autoCommand = setupAutonomousCommand();
      m_autoCommand = setupSimpleAuto();
    }
  }

  private void configureButtonBox() {
    m_buttonBox.intakeButton.whileHeld(m_ingestCargo.get());
    m_buttonBox.outgestButton.whileHeld(m_outgestCargo);

    m_buttonBox.centralizerUp.whileHeld(new RunCommand(m_centralizer::run, m_centralizer));
    m_buttonBox.centralizerDown.whileHeld(new RunCommand(m_centralizer::reverse, m_centralizer));
    m_buttonBox.chamberUp.whileHeld(new RunCommand(m_chamber::run, m_centralizer));
    m_buttonBox.chamberDown.whileHeld(new RunCommand(m_chamber::reverse, m_centralizer));

    m_buttonBox.shootButton.whenPressed(new InstantCommand(m_shooter::activate));
    m_buttonBox.shootButton.whenReleased(new InstantCommand(m_shooter::deactivate));
    m_buttonBox.turretTrackSwitch.whenPressed(new InstantCommand(m_turret::startTracking));
    m_buttonBox.turretTrackSwitch.whenReleased(new InstantCommand(m_turret::stopTracking));
    m_buttonBox.climbDirectionSwitch.whenPressed(m_hoodUp);
    m_buttonBox.climbDirectionSwitch.whenReleased(m_hoodDown);
    m_buttonBox.flywheelSwitch.whenPressed(new RunCommand(m_shooter::setFlywheelSpeedAuto, m_shooter));
    m_buttonBox.flywheelSwitch.whenReleased(m_stopFlywheel);

    m_buttonBox.stowClimberButton.whenPressed(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_STOW, false, () -> false));
    m_buttonBox.prepClimberButton.whenPressed(new ConditionalCommand(
      new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_PREP_LOW_MID, false, () -> false), 
      new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_PREP_HIGH_TRAVERSAL, false, () -> false),
      () -> m_buttonBox.getClimbBar() == ClimbBar.LOW || m_buttonBox.getClimbBar() == ClimbBar.MID
    ));
    m_buttonBox.raiseClimberButton.whenPressed(new ConditionalCommand(
      new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_RAISE_LOW, false, () -> false), 
      new ConditionalCommand(
        new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_RAISE_MID, false, () -> false),
        new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_RAISE_HIGH_TRAVERSAL, false, () -> false),
        () -> m_buttonBox.getClimbBar() == ClimbBar.MID
      ),
      () -> m_buttonBox.getClimbBar() == ClimbBar.LOW
    ));
    m_buttonBox.climbButton.whenPressed(new ConditionalCommand(
      new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_CLIMB_LOW, false, () -> false), 
      new ConditionalCommand(
        new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_CLIMB_MID, false, () -> false),
        new ConditionalCommand(
          new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_CLIMB_HIGH, true, m_buttonBox::isCancelClimbPressed),
          new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_CLIMB_TRAVERSAL, true, m_buttonBox::isCancelClimbPressed),
          () -> m_buttonBox.getClimbBar() == ClimbBar.HIGH
        ),
        () -> m_buttonBox.getClimbBar() == ClimbBar.MID
      ),
      () -> m_buttonBox.getClimbBar() == ClimbBar.LOW
    ));

    // m_buttonBox.pursueCargoButton.whenActive(m_pursueCargoCommand);
    // m_buttonBox.pursueCargoButton.whenReleased(m_stowIntakeAndShooter);
    // m_buttonBox.pursueCargoButton.whileHeld(m_allowRosCommandVelocities);
  }

  private void configureDashboardCommands() {
    // Drive testing commands
    SmartDashboard.putData("Drive forwards", new ArcadeDrive(m_drive, () -> 5, () -> 0, () -> m_drive.shiftToLow(), () -> Constants.MAX_SPEED_LOW));
    SmartDashboard.putData("Drive Basic", new TankDrive(m_drive, m_testController2::getLeftStickY, m_testController2::getRightStickY));

    // Trajectory testing commands
    SmartDashboard.putData("Ten Feet Forward", new AutoFollowTrajectory(m_drive, m_sensors, RapidReactTrajectories.generateTestTrajectory()));
    // SmartDashboard.putData("Barrel Run", new AutoFollowTrajectory(m_drive, m_sensors, RapidReactTrajectories.generateBarrelRunTrajectory()));
    // SmartDashboard.putData("Barrel Run 2", new AutoFollowTrajectory(m_drive, m_sensors, RapidReactTrajectories.generateBarrelRun2Trajectory()));

    // Intake testing commands
    SmartDashboard.putData("Intake:Ingest", m_ingestCargo.get());
    SmartDashboard.putData("Intake:Stow", m_stowIntakeTwo);

    // Centralizer and Chamber commmands
    SmartDashboard.putData("Centralizer:AcceptCargo", new FeederAcceptCargo(m_centralizer));
    SmartDashboard.putData("Centralizer:Run", new InstantCommand(m_centralizer::run, m_centralizer));
    SmartDashboard.putData("Centralizer:Reverse", new InstantCommand(m_centralizer::reverse, m_centralizer));
    SmartDashboard.putData("Centralizer:Stop", new InstantCommand(m_centralizer::stop, m_centralizer));
    SmartDashboard.putData("Centralizer:Cargolizer", m_centralizerCargolizer.get());
    
    SmartDashboard.putData("Chamber:AcceptCargo", new FeederAcceptCargo(m_chamber));
    SmartDashboard.putData("Chamber:Run", new InstantCommand(m_chamber::run, m_chamber));
    SmartDashboard.putData("Chamber:Reverse", new InstantCommand(m_chamber::reverse, m_chamber));
    SmartDashboard.putData("Chamber:Stop", new InstantCommand(m_chamber::stop, m_chamber));
    SmartDashboard.putData("Chamber:Cargolizer", m_chamberCargolizer.get());

    // Turret test commands
    SmartDashboard.putData("Turret Raw Control",new TurretRawJoystick(m_turret, m_testController2));
    SmartDashboard.putData("Turret Motion Magic Control",new TurretMotionMagicJoystick(m_turret, m_testController2));
    SmartDashboard.putData("Turret Go To 180", new InstantCommand(() -> {m_turret.goToFacing(180);}, m_turret));
    SmartDashboard.putData("Turret Go To 45", new InstantCommand(() -> {m_turret.goToFacing(45);}, m_turret));
    SmartDashboard.putData("Turret Go To 0", new InstantCommand(() -> {m_turret.goToFacing(0);}, m_turret));
    SmartDashboard.putData("Turret Go To -45", new InstantCommand(() -> {m_turret.goToFacing(-45);}, m_turret));
    SmartDashboard.putData("Turret Go To -180", new InstantCommand(() -> {m_turret.goToFacing(-180);}, m_turret));
    
    SmartDashboard.putData("Turret Track", new TurretTrack(m_turret, m_sensors.limelight));
    SmartDashboard.putData("Turret Activate Tracking", m_turretTrackingOn);
    SmartDashboard.putData("Turret Deactivate Tracking", new InstantCommand(m_turret::stopTracking));
    SmartDashboard.putData("Turret !!Calibrate!!", new TurretCalibrate(m_turret));
    SmartDashboard.putData("Turret Sync", new InstantCommand(m_turret::sync, m_turret));

    // Shooter testing commands
    SmartDashboard.putData("Shooter:Flywheel:RunRaw", m_startFlywheelRaw);
    SmartDashboard.putData("Shooter:Flywheel:RunSpeed", new RunCommand(() -> m_shooter.setFlywheelSpeed(p_shooterTestVelocity.getValue()), m_shooter));
    SmartDashboard.putData("Shooter:Flywheel:RunAuto", new RunCommand(m_shooter::setFlywheelSpeedAuto, m_shooter));
    SmartDashboard.putData("Shooter:Flywheel:StopRaw", m_stopFlywheelRaw);
    SmartDashboard.putData("Shooter:Flywheel:StopSpeed", m_stopFlywheel);
    SmartDashboard.putData("Shooter:Hood:Raise", new RunCommand(m_shooter::raiseHood, m_shooter));
    SmartDashboard.putData("Shooter:Hood:Lower", new RunCommand(m_shooter::lowerHood, m_shooter));
    SmartDashboard.putData("Shooter:Hood:UpRaw", new RunCommand(()->{m_shooter.setHoodPercentOut(1);}, m_shooter));
    SmartDashboard.putData("Shooter:Hood:DownRaw", new RunCommand(()->{m_shooter.setHoodPercentOut(-1);}, m_shooter));
    SmartDashboard.putData("Shooter:Hood:StopRaw", new RunCommand(()->{m_shooter.setHoodPercentOut(0);}, m_shooter));
    SmartDashboard.putData("Shooter:Activate", new InstantCommand(m_shooter::activate, m_shooter));
    SmartDashboard.putData("Shooter:Deactivate", new InstantCommand(m_shooter::deactivate, m_shooter));

    // Limelight
    SmartDashboard.putData("Limelight On", new LimelightToggle(m_sensors.limelight, true));
    SmartDashboard.putData("Limelight Off", new LimelightToggle(m_sensors.limelight, false));
    SmartDashboard.putData("Limelight Hood Up", new LimelightHoodToggle(m_sensors.limelight, true));
    SmartDashboard.putData("Limelight Hood Down", new LimelightHoodToggle(m_sensors.limelight, false));

    // Climber Commands
    SmartDashboard.putData(m_calibrateClimber);
    SmartDashboard.putData(m_manualModeClimber);
    SmartDashboard.putData(m_climberTestMotionMagic);
    SmartDashboard.putData(m_climberMotionMagicJoystick);

    SmartDashboard.putData(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_STOW, false, () -> false).withName("Climber M Stow"));
    SmartDashboard.putData(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_PREP_LOW_MID, false, () -> false).withName("Climber M Prep Low Mid"));
    SmartDashboard.putData(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_PREP_HIGH_TRAVERSAL, false, () -> false).withName("Climber M Prep High Traversal"));
    SmartDashboard.putData(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_RAISE_LOW, false, () -> false).withName("Climber M Raise Low"));
    SmartDashboard.putData(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_RAISE_MID, false, () -> false).withName("Climber M Raise Mid"));
    SmartDashboard.putData(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_RAISE_HIGH_TRAVERSAL, false, () -> false).withName("Climber M Raise High Traversal"));
    SmartDashboard.putData(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_CLIMB_LOW, false, () -> false).withName("Climber M Climb Low"));
    SmartDashboard.putData(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_CLIMB_MID, false, () -> false).withName("Climber M Climb Mid"));
    SmartDashboard.putData(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_CLIMB_HIGH ,true, m_buttonBox.cancelClimb).withName("Climber M Climb High"));
    SmartDashboard.putData(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_CLIMB_TRAVERSAL, true, m_buttonBox.cancelClimb).withName("Climber M Climb Traversal"));
  }

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(m_arcadeDrive);
    m_intake.setDefaultCommand(m_stowIntake);

    m_centralizer.setDefaultCommand(new FeederCargolizer(m_centralizer, m_intake, m_chamber));
    m_chamber.setDefaultCommand(new FeederCargolizer(m_chamber, m_centralizer, m_shooter));

    // m_shooter.setDefaultCommand(new RunCommand(m_shooter::setFlywheelSpeedAuto, m_shooter));
    m_turret.setDefaultCommand(new TurretTrack(m_turret, m_sensors.limelight));

    m_climber.setDefaultCommand( 
      new SequentialCommandGroup(
        new RunCommand(m_climber::calibrate, m_climber)
          .withInterrupt(m_climber::isCalibrated)
          .withName("calibrateClimber"),
          new RunCommand(() -> {}, m_climber)
      ));
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
