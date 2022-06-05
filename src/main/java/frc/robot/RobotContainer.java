// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.drive.TankDrive;
import frc.robot.commands.feeder.FeederAcceptCargo;
import frc.robot.commands.feeder.FeederCargolizer;
import frc.robot.commands.test.CollectLimelightRectification;
import frc.robot.commands.turret.HoodTrackCombo;
import frc.robot.commands.turret.ShooterTrackCombo;
import frc.robot.commands.turret.TurretCalibrate;
import frc.robot.commands.turret.TurretLock;
import frc.robot.commands.turret.TurretMotionMagicJoystick;
import frc.robot.commands.turret.TurretRawJoystick;
import frc.robot.commands.turret.TurretTrackLimelight;
import frc.robot.commands.turret.TurretTrackCombo;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Targeting;
import frc.robot.subsystems.Turret;
import frc.robot.util.CargoSource;
import frc.robot.util.NumberCache;
import frc.robot.util.RapidReactTrajectories;
import frc.robot.util.climber.ClimberConstants;
import frc.robot.util.controllers.ButtonBox;
import frc.robot.util.controllers.DriverController;
import frc.robot.util.controllers.FrskyDriverController;
import frc.robot.util.controllers.XboxController;
import frc.robot.util.controllers.ButtonBox.ClimbBar;
import frc.robot.util.controllers.ButtonBox.ClimbDirection;
import frc.robot.util.ThisRobotTable;
import frc.robot.commands.LimelightToggle;
import frc.robot.commands.ShootAll;
import frc.robot.commands.autos.AutoFollowTrajectory;
import frc.robot.commands.autos.DriveToCargo;
import frc.robot.commands.autos.DriveToWaypoint;
import frc.robot.commands.autos.DriveToWaypointWithHeading;
import frc.robot.commands.autos.DriveWithWaypointsPlan;
import frc.robot.commands.autos.PassthroughRosCommand;
import frc.robot.commands.autos.SetGlobalPoseToLimelight;
import frc.robot.commands.autos.SetGlobalPoseToWaypoint;
import frc.robot.commands.cameratilter.TiltCameraDown;
import frc.robot.commands.climber.ClimberMotionMagicJoystick;
import frc.robot.commands.climber.ClimberStateMachineExecutor;
import frc.robot.commands.climber.ClimberTestMotionMagic;
import frc.robot.commands.climber.ManualModeClimber;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.DriveDegrees;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PreferenceConstants;

public class RobotContainer {
  /////////////////////////////////////////////////////////////////////////////
  //                              SUBSYSTEMS                                 //
  /////////////////////////////////////////////////////////////////////////////
  private final Sensors m_sensors = new Sensors();
  private final Drive m_drive = new Drive(m_sensors);
  private final Climber m_climber = new Climber(m_sensors::isCoastButtonPressed);
  private final Intake m_intake = new Intake();
  private final Turret m_turret = new Turret(m_sensors);
  private final Hood m_hood = new Hood(m_sensors);
  private final Feeder m_centralizer = new Feeder("Centralizer", Constants.FEEDER_CENTRALIZER_MOTOR_ID, true, Constants.FEEDER_CENTRALIZER_BLOCKER_ID);
  private final Feeder m_chamber = new Feeder("Chamber", Constants.FEEDER_CHAMBER_MOTOR_ID, false);
  private final ThisRobotTable m_ros_interface = new ThisRobotTable(
    m_drive,
    Robot.isSimulation() ? Constants.COPROCESSOR_ADDRESS_SIMULATED : Constants.COPROCESSOR_ADDRESS,
    Constants.COPROCESSOR_PORT,
    Constants.COPROCESSOR_TABLE_UPDATE_DELAY,
    m_climber.outerArm, m_climber.innerArm, m_intake, m_turret, m_sensors, m_hood
  );
  private final Shooter m_shooter = new Shooter(m_sensors, m_hood, m_drive, m_turret, new CargoSource[]{m_chamber, m_centralizer}, m_ros_interface);
  private final Navigation m_nav = new Navigation(m_ros_interface);

  private final Targeting m_targeting = new Targeting(m_sensors.limelight, m_ros_interface, m_turret, m_hood, m_drive, m_shooter);

  /////////////////////////////////////////////////////////////////////////////
  //                              CONTROLLERS                                //
  /////////////////////////////////////////////////////////////////////////////
  private final DriverController m_driverController = new FrskyDriverController(Constants.DRIVER_CONTROLLER_ID);
  private final ButtonBox m_buttonBox = new ButtonBox(Constants.BUTTON_BOX_ID);
  private final XboxController m_testController = new XboxController(Constants.TEST_CONTROLLER_ID);
  private final XboxController m_testController2 = new XboxController(Constants.TEST_CONTROLLER_2_ID);
  
  private DoublePreferenceConstant p_shooterAutoSpeed = new DoublePreferenceConstant("Shooter Auto Speed", 0.0);

  private boolean m_hasConfiguredDashboardButtons = false;

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
              m_drive.autoshift();
            }
          }, 
          () -> m_driverController.getForceLowGear() ? Constants.MAX_SPEED_LOW : Constants.MAX_SPEED_HIGH);


  /////////////////////////////////////
  //          BALL HANDLING          //
  /////////////////////////////////////

  private CommandBase m_ingestCargo = new RunCommand(() -> {
        m_intake.deploy();
        m_intake.rollerIntake();
      }, m_intake);

  private CommandBase m_outgestCargo = new ParallelCommandGroup(new RunCommand(() -> {
        m_intake.deploy();
        m_intake.rollerOutgest();
      }, m_intake),
      new RunCommand(m_chamber::forceReverse, m_chamber),
      new RunCommand(m_centralizer::forceReverse, m_centralizer));

  private CommandBase m_stowIntake = new RunCommand(() -> {
        m_intake.stow();
        m_intake.rollerStop();
      }, m_intake);

  private CommandBase m_centralizerCargolizer = new FeederCargolizer(m_centralizer, m_intake, m_chamber);
  private CommandBase m_chamberCargolizer = new FeederCargolizer(m_chamber, m_centralizer, m_shooter);

  /////////////////////////////////////
  //            SHOOTING             //
  /////////////////////////////////////

  private CommandBase m_startFlywheel = new ShooterTrackCombo(m_shooter, m_targeting);
  private CommandBase m_stopFlywheel = new InstantCommand(() -> {m_shooter.setFlywheelSpeed(0.0);}, m_shooter);
  private CommandBase m_flywheelFenderShot = new RunCommand(m_shooter::setFlywheelFenderShot, m_shooter);

  private CommandBase m_hoodUp = new RunCommand(m_hood::raiseHood, m_hood);
  private CommandBase m_hoodMid = new RunCommand(m_hood::midHood, m_hood);
  private CommandBase m_hoodDown = new RunCommand(m_hood::lowerHood, m_hood);
  private CommandBase m_hoodAuto = new HoodTrackCombo(m_hood, m_targeting);

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

  private CommandBase m_autoCommand = new WaitCommand(15);
  private String m_autoCommandName = "Wait 1";

  private DoublePreferenceConstant p_oneBallDelay = new DoublePreferenceConstant("One Ball Delay", 0.25);
  private Timer m_oneBallTimer = new Timer();
  
  private CommandBase m_autoOneBallLeft =
  new ParallelCommandGroup(
    new TiltCameraDown(m_sensors),
    new InstantCommand(m_turret::startTracking),
    new InstantCommand(m_sensors.limelight::ledOn),
    new InstantCommand(() -> m_turret.setDefaultFacing(0)),
    new SequentialCommandGroup(
      new ParallelCommandGroup(
        new RunCommand(() -> {m_intake.stow(); m_intake.rollerStop();}, m_intake),
        new SequentialCommandGroup(
          new WaitUntilCommand(() -> m_oneBallTimer.hasElapsed(p_oneBallDelay.getValue())),
          new AutoFollowTrajectory(m_drive, RapidReactTrajectories.generatePathWeaverTrajectory("OneBall.wpilib.json"), true),
          new WaitCommand(0.5),
          new ShootAll(m_shooter).withTimeout(3.0)
        )
      ),
      new ParallelCommandGroup(
        new RunCommand(() -> {m_intake.deploy(); m_intake.rollerIntake();}, m_intake),
        new SequentialCommandGroup(
          new InstantCommand(() -> m_targeting.enableDefault(80, 180)),
          new WaitCommand(1),
          new ShootAll(m_shooter).withTimeout(3.0),
          new DriveToCargo(m_nav, m_ros_interface, m_drive, m_shooter, m_sensors, 5)
        )
      )
    )
  );

  private CommandBase m_autoOneBallRight =
  new ParallelCommandGroup(
    new TiltCameraDown(m_sensors),
    new InstantCommand(m_turret::startTracking),
    new InstantCommand(m_sensors.limelight::ledOn),
    new InstantCommand(() -> m_turret.setDefaultFacing(0)),
    new SequentialCommandGroup(
      new ParallelCommandGroup(
        new RunCommand(() -> {m_intake.stow(); m_intake.rollerStop();}, m_intake),
        new SequentialCommandGroup(
          new WaitUntilCommand(() -> m_oneBallTimer.hasElapsed(p_oneBallDelay.getValue())),
          new AutoFollowTrajectory(m_drive, RapidReactTrajectories.generatePathWeaverTrajectory("OneBall.wpilib.json"), true),
          new WaitCommand(0.5),
          new ShootAll(m_shooter).withTimeout(3.0)
        )
      ),
      new ParallelCommandGroup(
        new RunCommand(() -> {m_intake.deploy(); m_intake.rollerIntake();}, m_intake),
        new SequentialCommandGroup(
          new InstantCommand(() -> m_targeting.enableDefault(80, -135)),
          new WaitCommand(1),
          new ShootAll(m_shooter).withTimeout(3.0),
          new DriveToCargo(m_nav, m_ros_interface, m_drive, m_shooter, m_sensors, 5)
        )
      )
    )
  );

  private CommandBase m_autoTwoBall = 
  new ParallelCommandGroup(
    new TiltCameraDown(m_sensors),
    new InstantCommand(m_turret::startTracking),
    new InstantCommand(m_sensors.limelight::ledOn),
    new InstantCommand(() -> m_turret.setDefaultFacing(0)),
    new RunCommand(() -> {m_intake.deploy(); m_intake.rollerIntake();}, m_intake),
    // new SetGlobalPoseToWaypoint(m_nav, "<team>_start_2"),
    new HoodTrackCombo(m_hood, m_targeting),
    new SequentialCommandGroup(
      // new DriveDistanceMeters(m_drive, 1.5, 5.0),
      new AutoFollowTrajectory(m_drive, RapidReactTrajectories.generatePathWeaverTrajectory("Boring.wpilib.json"), true),
      new WaitCommand(0.5),
      new ShootAll(m_shooter).withTimeout(4.0),
      new AutoFollowTrajectory(m_drive, RapidReactTrajectories.generatePathWeaverTrajectory("Spicy.wpilib.json"), false),
      new ShootAll(m_shooter).withTimeout(3.0),
      new DriveDegrees(m_drive, 150.0, 180.0),
      new DriveToCargo(m_nav, m_ros_interface, m_drive, m_shooter, m_sensors, 5).withInterrupt(m_chamber::hasCargo),
      new InstantCommand(m_shooter::activatePermissive)
    )
  );

  private CommandBase m_autoPursuitOnly = 
  new ParallelCommandGroup(
    new TiltCameraDown(m_sensors),
    new RunCommand(() -> {m_intake.deploy(); m_intake.rollerIntake();}, m_intake),
    new DriveToCargo(m_nav, m_ros_interface, m_drive, m_shooter, m_sensors, 5)
  );

  private CommandBase m_autoTwoBallSpicy = 
  new ParallelCommandGroup(
    new TiltCameraDown(m_sensors),
    new InstantCommand(m_turret::startTracking),
    new InstantCommand(m_sensors.limelight::ledOn),
    new InstantCommand(() -> m_turret.setDefaultFacing(0)),
    new RunCommand(() -> {m_intake.deploy(); m_intake.rollerIntake();}, m_intake),
    new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new InstantCommand(() -> m_targeting.setModeToLimelight()),
          new WaitCommand(0.5),
          new AutoFollowTrajectory(m_drive, RapidReactTrajectories.generatePathWeaverTrajectory("Boring.wpilib.json"), true),
          new WaitCommand(0.5),
          new ShootAll(m_shooter).withTimeout(4.0),
          new AutoFollowTrajectory(m_drive, RapidReactTrajectories.generatePathWeaverTrajectory("Spicy.wpilib.json"), false),
          new InstantCommand(m_turret::stopTracking),
          new InstantCommand(m_sensors.limelight::ledOff),
          new ShootAll(m_shooter).withTimeout(2.0),
          new InstantCommand(m_turret::startTracking),
          new InstantCommand(m_sensors.limelight::ledOn),
          new DriveDegrees(m_drive, -120.0, -120.0),
          new InstantCommand(() -> m_targeting.setModeToDefault())
        ),
        new RunCommand(m_hood::raiseHood, m_hood)
      ),
      new ParallelCommandGroup(
        new HoodTrackCombo(m_hood, m_targeting),
        new SequentialCommandGroup(
          new DriveToCargo(m_nav, m_ros_interface, m_drive, m_shooter, m_sensors, 5).withInterrupt(m_chamber::hasCargo),
          new InstantCommand(m_shooter::activatePermissive)
        )
      )
    )
  );

  private CommandBase m_autoFiveBall = 
    new ParallelCommandGroup(
      new TiltCameraDown(m_sensors),
      new InstantCommand(m_turret::startTracking),
      new InstantCommand(m_sensors.limelight::ledOn),
      new InstantCommand(() -> m_turret.setDefaultFacing(0)),
      new InstantCommand(() -> m_targeting.setModeToLimelight()),
      new RunCommand(() -> {m_intake.deploy(); m_intake.rollerIntake();}, m_intake),
      new SequentialCommandGroup(
        // new InstantCommand(() -> m_targeting.disableDefault()),
        new InstantCommand(() -> m_targeting.enableDefault(97, -14.5)),
        new WaitCommand(0.5),
        new ShootAll(m_shooter).withTimeout(3.0),
        new AutoFollowTrajectory(m_drive, RapidReactTrajectories.generatePathWeaverTrajectory("legone.wpilib.json"), true),
        // new InstantCommand(() -> m_targeting.enableDefault(159, 30)),
        new InstantCommand(m_targeting::disableDefault),
        new AutoFollowTrajectory(m_drive, RapidReactTrajectories.generatePathWeaverTrajectory("legtwo.wpilib.json"), false),
        new AutoFollowTrajectory(m_drive, RapidReactTrajectories.generatePathWeaverTrajectory("legthree.wpilib.json"), false),
        new ShootAll(m_shooter).withTimeout(3.0),
        new InstantCommand(m_targeting::disableDefault),
        new AutoFollowTrajectory(m_drive, RapidReactTrajectories.generatePathWeaverTrajectory(
          DriverStation.getAlliance() == Alliance.Red ? "legfour.wpilib.json" : "legfour.wpilib.json"), false),
        new WaitCommand(0.3),
        // new InstantCommand(() -> m_targeting.enableDefault(226, 25)),
        new AutoFollowTrajectory(m_drive, RapidReactTrajectories.generatePathWeaverTrajectory(
          DriverStation.getAlliance() == Alliance.Red ? "legsix.wpilib.json" : "legsix.wpilib.json"), true),
        new InstantCommand(m_shooter::activatePermissive)
      )
    );

    private CommandBase m_autoTwoBallMysterySpice = 
    new ParallelCommandGroup(
      new TiltCameraDown(m_sensors),
      new InstantCommand(m_turret::startTracking),
      new InstantCommand(m_sensors.limelight::ledOn),
      new InstantCommand(() -> m_turret.setDefaultFacing(0)),
      new InstantCommand(() -> m_targeting.setModeToLimelight()),
      new SequentialCommandGroup(
        new WaitCommand(0.5),
        new ParallelDeadlineGroup(
          new SequentialCommandGroup(
            new ShootAll(m_shooter).withTimeout(4.0),
            new AutoFollowTrajectory(m_drive, RapidReactTrajectories.generatePathWeaverTrajectory("mysteryspice.wpilib.json"), true)
          ),
          new RunCommand(() -> {m_intake.deploy(); m_intake.rollerIntake();}, m_intake),
          new FeederCargolizer(m_chamber, m_centralizer, m_shooter),
          new FeederCargolizer(m_centralizer, m_intake, m_chamber)
        ),
        new WaitCommand(0.5),
        new ParallelDeadlineGroup(
          new WaitCommand(4.0),
          new RunCommand(() -> {m_intake.stow(); m_intake.rollerStop();}, m_intake),
          new FeederCargolizer(m_chamber, m_centralizer, m_shooter),
          new SequentialCommandGroup(
            new ParallelDeadlineGroup(
              new WaitCommand(0.5),
              new RunCommand(m_centralizer::stop, m_centralizer)
            ),
            new RunCommand(m_centralizer::forceReverse, m_centralizer)
          ),
          new SequentialCommandGroup(
            new WaitCommand(0.75),
            new RunCommand(m_shooter::activatePermissive)
          )
        ),
        new InstantCommand(m_shooter::deactivate),
        new WaitCommand(0.5),
        new ParallelDeadlineGroup(
            new AutoFollowTrajectory(m_drive, RapidReactTrajectories.generatePathWeaverTrajectory("extraspicy.wpilib.json"), true),
            new RunCommand(() -> {m_intake.deploy(); m_intake.rollerIntake();}, m_intake),
            new FeederCargolizer(m_chamber, m_centralizer, m_shooter),
            new FeederCargolizer(m_centralizer, m_intake, m_chamber)
          ),
        new ParallelDeadlineGroup(
          new WaitCommand(4.0),
          new RunCommand(() -> {m_intake.stow(); m_intake.rollerStop();}, m_intake),
          new SequentialCommandGroup(
            new ParallelDeadlineGroup(
              new WaitCommand(0.5),
              new RunCommand(m_chamber::stop, m_chamber),
              new RunCommand(m_centralizer::stop, m_centralizer)
            ),
            new ParallelCommandGroup(
              new RunCommand(m_chamber::forceReverse, m_chamber),
              new RunCommand(m_centralizer::forceReverse, m_centralizer)
            )
          )
        )
      )
    );


  public static  String getTeamColorName() {
    if (DriverStation.getAlliance() == Alliance.Red) {
      return "red";
    }
    else {
      return "blue";
    }
  }
  
  /////////////////////////////////////////////////////////////////////////////
  //                                 SETUP                                   //
  /////////////////////////////////////////////////////////////////////////////

  public RobotContainer(Robot robot) {
    SmartDashboard.putBoolean("Publishing Enabled", false);
    LiveWindow.disableAllTelemetry();

    configurePeriodics(robot);
    configureButtonBox();
    configureDefaultCommands();
  }

  private void configurePeriodics(Robot robot) {
    robot.addPeriodic(m_ros_interface::update, Constants.COPROCESSOR_PERIODIC_UPDATE_DELAY, Constants.COPROCESSOR_PERIODIC_UPDATE_OFFSET);
    robot.addPeriodic(m_ros_interface::updateSlow, Constants.COPROCESSOR_SLOW_PERIODIC_UPDATE_DELAY, Constants.COPROCESSOR_SLOW_PERIODIC_UPDATE_OFFSET);

    robot.addPeriodic(PreferenceConstants::update, 1);
    robot.addPeriodic(
      () -> {
        if (isPublishingEnabled() && !m_hasConfiguredDashboardButtons) {
          configureDashboardCommands();
          m_hasConfiguredDashboardButtons = true;
        }
      }, 5);
  }

  public static boolean isPublishingEnabled() {
    if (NumberCache.hasValue("Publishing Enabled")) {
      return NumberCache.getValue("Publishing Enabled") > 0;
    }

    return NumberCache.pushValue("Publishing Enabled", SmartDashboard.getBoolean("Publishing Enabled", false) ? 1 : -1) > 0;
  }
  
  public void disabledPeriodic() {
    if (m_buttonBox.isShootButtonPressed() && !m_autoCommandName.equals("5 Cargo")) {
      m_autoCommand = m_autoFiveBall;
      m_autoCommandName = "5 Cargo";
      new SetGlobalPoseToWaypoint(m_nav, "<team>_start_5").schedule();

    }

    if (m_buttonBox.isChamberUpButtonPressed() && !m_autoCommandName.equals("2 Cargo")) {
      m_autoCommand = m_autoTwoBall;
      m_autoCommandName = "2 Cargo";
      new SetGlobalPoseToWaypoint(m_nav, "<team>_start_2").schedule();
    }

    if (m_buttonBox.isChamberDownButtonPressed() && !m_autoCommandName.equals("Mystery Spice")) {
      m_autoCommand = m_autoTwoBallMysterySpice;
      m_autoCommandName = "Mystery Spice";
      new SetGlobalPoseToWaypoint(m_nav, "<team>_start_2").schedule();
    }

    if (m_buttonBox.isCentralizerUpButtonPressed() && !m_autoCommandName.equals("Wait 1")) {
      m_autoCommand = new WaitCommand(1.0);
      m_autoCommandName = "Wait 1";
      new SetGlobalPoseToWaypoint(m_nav, "center").schedule();
    }

    if (m_buttonBox.isDefaultTurretButtonPressed() && !m_autoCommandName.equals("1 Ball Left")) {
      m_autoCommand = m_autoOneBallLeft;
      m_autoCommandName = "1 Ball Left";
      new SetGlobalPoseToWaypoint(m_nav, "<team>_start_1L").schedule();
    }

    if (m_buttonBox.isCancelClimbPressed() && !m_autoCommandName.equals("1 Ball Left")) {
      m_autoCommand = m_autoOneBallRight;
      m_autoCommandName = "1 Ball Right";
      new SetGlobalPoseToWaypoint(m_nav, "<team>_start_1R").schedule();
    }

    if (m_buttonBox.isOutgestButtonPressed() && !m_autoCommandName.equals("Pursuit Only")) {
      m_autoCommand = m_autoPursuitOnly;
      m_autoCommandName = "Pursuit Only";
    }

    if (m_buttonBox.isCentralizerDownButtonPressed()) {
      m_sensors.limelight.ledOn();
    } else {
      m_sensors.limelight.ledOff();
    }

    SmartDashboard.putString("Auto", m_autoCommandName);
  }

  public void teleopInit() {
    
    if (m_driverController.getShooterMode()) {
      m_shooter.activatePermissive();
    } else if (m_buttonBox.isAutoShootSwitchOn()) {
      m_shooter.activateRestrictive();
    } else {
      m_shooter.deactivate();
    }
    
    if (m_buttonBox.isTrackTurretSwitchOn()) {
      m_turret.startTracking();
      m_startFlywheel.schedule();
      m_hoodAuto.schedule();
      
    } else {
      m_turret.stopTracking();
      m_flywheelFenderShot.schedule();
      m_hoodUp.schedule();
    }

    m_turret.setDefaultFacing(0.);
    m_targeting.disableDefault();
    m_ros_interface.enableCargoMarauding();
    // m_ros_interface.disableCargoMarauding();
  }

  public void robotFirstPeriodic() {
    if (!m_nav.isConnected() || m_buttonBox.isROSDisableSwitchOn() || DriverStation.isAutonomous()) {
      m_targeting.setModeToLimelight();
    } else if (!DriverStation.isAutonomous()) {
      m_targeting.setModeToDefault();
    }

    m_sensors.firstPeriodic();
    m_targeting.firstPeriodic();
  }

  private void configureButtonBox() {
    Button shooterButton = new Button(m_driverController::getShooterMode);
    shooterButton.whenPressed(new InstantCommand(m_shooter::activatePermissive));
    shooterButton.whenReleased(new ConditionalCommand(new InstantCommand(m_shooter::activateRestrictive), new InstantCommand(m_shooter::deactivate), m_buttonBox::isAutoShootSwitchOn));

    m_buttonBox.intakeButton.whileHeld(m_ingestCargo);
    m_buttonBox.outgestButton.whileHeld(m_outgestCargo);

    m_buttonBox.centralizerUp.whileHeld(new RunCommand(m_centralizer::forceForwards, m_centralizer));
    m_buttonBox.centralizerDown.whileHeld(new RunCommand(m_centralizer::forceReverse, m_centralizer));
    m_buttonBox.chamberUp.whileHeld(new RunCommand(m_chamber::forceForwards, m_chamber));
    m_buttonBox.chamberDown.whileHeld(new RunCommand(m_chamber::forceReverse, m_chamber));

    m_buttonBox.shootButton.whenPressed(new InstantCommand(m_shooter::activatePermissive));
    m_buttonBox.shootButton.whenReleased(new ConditionalCommand(new InstantCommand(m_shooter::activateRestrictive), new InstantCommand(m_shooter::deactivate), m_buttonBox::isAutoShootSwitchOn));
    m_buttonBox.autoShootSwitch.whenPressed(new InstantCommand(m_shooter::activateRestrictive));
    m_buttonBox.autoShootSwitch.whenReleased(new ConditionalCommand(new InstantCommand(m_shooter::activatePermissive), new InstantCommand(m_shooter::deactivate), m_buttonBox::isShootButtonPressed));


    m_buttonBox.turretTrackSwitch.whenPressed(new InstantCommand(m_turret::startTracking));
    m_buttonBox.turretTrackSwitch.whenPressed(m_startFlywheel);
    m_buttonBox.turretTrackSwitch.whenPressed(m_hoodAuto);
    m_buttonBox.turretTrackSwitch.whenReleased(new InstantCommand(m_turret::stopTracking));
    m_buttonBox.turretTrackSwitch.whenReleased(m_flywheelFenderShot);
    m_buttonBox.turretTrackSwitch.whenReleased(m_hoodUp);
    
    // m_buttonBox.rosDisableSwitch.whenPressed(m_arcadeDrive);
    // m_buttonBox.rosDisableSwitch.whenReleased(new PassthroughRosCommand(m_drive, m_ros_interface));
    
    m_buttonBox.rosDisableSwitch.whenPressed(new InstantCommand(m_targeting::setModeToLimelight) {
      @Override
      public boolean runsWhenDisabled() {
        return true;
      }
    });

    m_buttonBox.rosDisableSwitch.whenReleased(new InstantCommand(m_targeting::setModeToDefault) {
      @Override
      public boolean runsWhenDisabled() {
        return true;
      }
    });

    // Enable cargo marauding
    m_buttonBox.climbDirectionSwitch.whenPressed(
      new InstantCommand(m_ros_interface::enableCargoMarauding) {
        @Override
        public boolean runsWhenDisabled() {
          return true;
        }
      }
    );
    // Disable cargo marauding
    m_buttonBox.climbDirectionSwitch.whenReleased(
      new InstantCommand(m_ros_interface::disableCargoMarauding) {
        @Override
        public boolean runsWhenDisabled() {
          return true;
        }
      }
    );

    // m_buttonBox.rosDisableSwitch.whenPressed(new InstantCommand(m_ros_interface::stopComms) {
    //   @Override
    //   public boolean runsWhenDisabled() {
    //     return true;
    //   }
    // });
    // m_buttonBox.rosDisableSwitch.whenReleased(new InstantCommand(m_ros_interface::startComms) {
    //   @Override
    //   public boolean runsWhenDisabled() {
    //     return true;
    //   }
    // });
    m_buttonBox.defaultTurretButton.whenPressed(new InstantCommand(() -> m_turret.setDefaultFacing(180.)));
    m_buttonBox.defaultTurretButton.whenReleased(new InstantCommand(() -> m_turret.setDefaultFacing(0.)));

    m_buttonBox.stowClimberButton.whenPressed(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_STOW));
    m_buttonBox.prepClimberButton.whenPressed(new ParallelCommandGroup(
      new InstantCommand(() -> {m_turret.setDefaultFacing(0.);}),
      new InstantCommand(m_turret::stopTracking),
      new RunCommand(m_hood::lowerHood, m_hood),
      new RunCommand(() -> m_shooter.setFlywheelSpeed(0), m_shooter),
      new SequentialCommandGroup(
        new WaitUntilCommand(() -> m_hood.isDown() && m_turret.isSafeForClimber()),
        new ConditionalCommand(
          new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_PREP_LOW_MID), 
          new ConditionalCommand(
            new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_PREP_HIGH_TRAVERSAL_FORWARDS),
            new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_PREP_HIGH_TRAVERSAL_REVERSE),
            () -> m_buttonBox.getClimbBar() == ClimbBar.HIGH
          ),
          () -> m_buttonBox.getClimbBar() == ClimbBar.LOW || m_buttonBox.getClimbBar() == ClimbBar.MID
        )
      )
    ));
    m_buttonBox.raiseClimberButton.whenPressed(new ParallelCommandGroup(
      new InstantCommand(() -> {m_turret.setDefaultFacing(0.);}),
      new InstantCommand(m_turret::stopTracking),
      new RunCommand(m_hood::lowerHood, m_hood),
      new RunCommand(() -> m_shooter.setFlywheelSpeed(0), m_shooter),
      new SequentialCommandGroup(
        new WaitUntilCommand(() -> m_hood.isDown() && m_turret.isSafeForClimber()),
        new ConditionalCommand(
          new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_RAISE_LOW), 
          new ConditionalCommand(
            new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_RAISE_MID),
            new ConditionalCommand(
              new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_RAISE_HIGH_TRAVERSAL_FORWARDS),
              new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_RAISE_HIGH_TRAVERSAL_REVERSE),
              () -> m_buttonBox.getClimbBar() == ClimbBar.HIGH
            ),
            () -> m_buttonBox.getClimbBar() == ClimbBar.MID
          ),
          () -> m_buttonBox.getClimbBar() == ClimbBar.LOW
        )
      )
    ));
    m_buttonBox.climbButton.whenPressed(new ParallelCommandGroup(
      new InstantCommand(m_turret::stopTracking),
      new RunCommand(m_hood::lowerHood, m_hood),
      new RunCommand(() -> m_shooter.setFlywheelSpeed(0), m_shooter),
      new SequentialCommandGroup(
        new WaitUntilCommand(() -> m_hood.isDown() && m_turret.isSafeForClimber()),
        new ConditionalCommand(
          new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_CLIMB_LOW), 
          new ConditionalCommand(
            new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_CLIMB_MID),
            new ConditionalCommand(
                new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_CLIMB_HIGH_FORWARDS, true, () -> false, m_buttonBox::isCancelClimbPressed),
                new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_CLIMB_TRAVERSAL_REVERSE, true, m_buttonBox::isCancelClimbPressed, () -> false),
              () -> m_buttonBox.getClimbBar() == ClimbBar.HIGH
            ),
            () -> m_buttonBox.getClimbBar() == ClimbBar.MID
          ),
          () -> m_buttonBox.getClimbBar() == ClimbBar.LOW
        )
      )
    ));
  }

  private void configureDashboardCommands() {
    // Drive testing commands
    SmartDashboard.putData("Drive Basic Tank", new TankDrive(m_drive, m_testController2::getLeftStickY, m_testController2::getRightStickY));

    // Autonomous testing
    SmartDashboard.putData("Auto Two Ball", m_autoTwoBall);
    SmartDashboard.putData("Auto Five Ball", m_autoFiveBall);
    SmartDashboard.putData("Tilt Camera Down", new TiltCameraDown(m_sensors));

    // Trajectory testing
    SmartDashboard.putData("Ten Feet Trajectory", new AutoFollowTrajectory(m_drive, RapidReactTrajectories.generateStraightTrajectory(10.0), true));
    SmartDashboard.putData("Five Ball Trajectory", new AutoFollowTrajectory(m_drive, RapidReactTrajectories.generatePathWeaverTrajectory("ThreeForThreeInThree.wpilib.json"), true));

    // Intake testing commands
    SmartDashboard.putData("Intake:Ingest", m_ingestCargo);
    SmartDashboard.putData("Intake:Outgest", m_outgestCargo);
    SmartDashboard.putData("Intake:Stow", m_stowIntake);

    // Centralizer and Chamber commmands
    SmartDashboard.putData("Centralizer:AcceptCargo", new FeederAcceptCargo(m_centralizer));
    SmartDashboard.putData("Centralizer:Run", new RunCommand(m_centralizer::forceForwards, m_centralizer));
    SmartDashboard.putData("Centralizer:Reverse", new RunCommand(m_centralizer::forceReverse, m_centralizer));
    SmartDashboard.putData("Centralizer:Stop", new RunCommand(m_centralizer::stop, m_centralizer));
    SmartDashboard.putData("Centralizer:Cargolizer", m_centralizerCargolizer);
    
    SmartDashboard.putData("Chamber:AcceptCargo", new FeederAcceptCargo(m_chamber));
    SmartDashboard.putData("Chamber:Run", new RunCommand(m_chamber::forceForwards, m_chamber));
    SmartDashboard.putData("Chamber:Reverse", new RunCommand(m_chamber::forceReverse, m_chamber));
    SmartDashboard.putData("Chamber:Stop", new RunCommand(m_chamber::stop, m_chamber));
    SmartDashboard.putData("Chamber:Cargolizer", m_chamberCargolizer);

    // Turret test commands
    SmartDashboard.putData("Turret Raw Control",new TurretRawJoystick(m_turret, m_testController2));
    SmartDashboard.putData("Turret Motion Magic Control",new TurretMotionMagicJoystick(m_turret, m_testController2));
    SmartDashboard.putData("Turret Go To 180", new RunCommand(() -> {m_turret.goToFacing(180);}, m_turret));
    SmartDashboard.putData("Turret Go To 45", new RunCommand(() -> {m_turret.goToFacing(45);}, m_turret));
    SmartDashboard.putData("Turret Go To 0", new RunCommand(() -> {m_turret.goToFacing(0);}, m_turret));
    SmartDashboard.putData("Turret Go To -45", new RunCommand(() -> {m_turret.goToFacing(-45);}, m_turret));
    SmartDashboard.putData("Turret Go To -180", new RunCommand(() -> {m_turret.goToFacing(-180);}, m_turret));
    
    SmartDashboard.putData("Turret Track", new TurretTrackLimelight(m_turret, m_sensors.limelight));
    SmartDashboard.putData("Turret Activate Tracking", new InstantCommand(m_turret::startTracking));
    SmartDashboard.putData("Turret Deactivate Tracking", new InstantCommand(m_turret::stopTracking));
    SmartDashboard.putData("Turret Set Motion Offset", new InstantCommand(() -> {m_sensors.limelight.setMotionOffset(new DoublePreferenceConstant("Auto Motion Offset", 0.0).getValue());}));
    SmartDashboard.putData("Turret Clear Motion Offset", new InstantCommand(() -> {m_sensors.limelight.setMotionOffset(0.0);}));

    SmartDashboard.putData("Turret !!Calibrate!!", new TurretCalibrate(m_turret));
    SmartDashboard.putData("Turret Sync", new InstantCommand(m_turret::sync, m_turret));

    // Camera tilter test commands
    SmartDashboard.putData("Camera go to level", new RunCommand(() -> {m_sensors.setCameraTilterAngle(Constants.CAMERA_TILT_LEVEL_ANGLE);}, m_sensors));
    SmartDashboard.putData("Camera go to down", new RunCommand(() -> {m_sensors.setCameraTilterAngle(Constants.CAMERA_TILT_DOWN_ANGLE);}, m_sensors));
    SmartDashboard.putData("Camera go to up", new RunCommand(() -> {m_sensors.setCameraTilterAngle(Constants.CAMERA_TILT_UP_ANGLE);}, m_sensors));
    SmartDashboard.putData("Camera go to 0.0", new RunCommand(() -> {m_sensors.setCameraTilterAngle(0.0);}, m_sensors));
    SmartDashboard.putData("Camera go to 180.0", new RunCommand(() -> {m_sensors.setCameraTilterAngle(180.0);}, m_sensors));
    SmartDashboard.putData("Camera go to 90.0", new RunCommand(() -> {m_sensors.setCameraTilterAngle(90.0);}, m_sensors));

    // Shooter testing commands
    SmartDashboard.putData("Shooter:Flywheel:RunRaw", new InstantCommand(() -> {m_shooter.setFlywheelRaw(new DoublePreferenceConstant("Shooter Test Output", 0.0).getValue());}, m_shooter));
    SmartDashboard.putData("Shooter:Flywheel:StopRaw", new InstantCommand(() -> {m_shooter.setFlywheelRaw(0.0);}, m_shooter));
    SmartDashboard.putData("Shooter:Flywheel:RunSpeed", new InstantCommand(() -> {m_shooter.setFlywheelSpeed(new DoublePreferenceConstant("Shooter Test Speed", 0.0).getValue());}, m_shooter));
    SmartDashboard.putData("Shooter:Flywheel:RunAuto", m_startFlywheel);
    SmartDashboard.putData("Shooter:Flywheel:StopSpeed", m_stopFlywheel);
    SmartDashboard.putData("Shooter:ActivatePermissive", new InstantCommand(m_shooter::activatePermissive, m_shooter));
    SmartDashboard.putData("Shooter:ActivateRestrictive", new InstantCommand(m_shooter::activateRestrictive, m_shooter));
    SmartDashboard.putData("Shooter:Deactivate", new InstantCommand(m_shooter::deactivate, m_shooter));

    SmartDashboard.putData("Hood:Raise", m_hoodUp);
    SmartDashboard.putData("Hood:Mid", m_hoodMid);
    SmartDashboard.putData("Hood:Lower", m_hoodDown);
    SmartDashboard.putData("Hood:Auto", m_hoodAuto);
    SmartDashboard.putData("Hood:UpRaw", new RunCommand(()->{m_hood.setHoodPercentOut(1);}, m_hood));
    SmartDashboard.putData("Hood:DownRaw", new RunCommand(()->{m_hood.setHoodPercentOut(-1);}, m_hood));
    SmartDashboard.putData("Hood:StopRaw", new RunCommand(()->{m_hood.setHoodPercentOut(0);}, m_hood));

    SmartDashboard.putData("Rectify Limelight", new CollectLimelightRectification(m_sensors.limelight, m_ros_interface));

    // Limelight
    SmartDashboard.putData("Limelight On", new LimelightToggle(m_sensors.limelight, true));
    SmartDashboard.putData("Limelight Off", new LimelightToggle(m_sensors.limelight, false));

    // Climber Commands
    SmartDashboard.putData(m_calibrateClimber);
    SmartDashboard.putData(m_manualModeClimber);
    SmartDashboard.putData(m_climberTestMotionMagic);
    SmartDashboard.putData(m_climberMotionMagicJoystick);

    SmartDashboard.putData(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_STOW).withName("Climber M Stow"));
    SmartDashboard.putData(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_PREP_LOW_MID).withName("Climber M Prep Low Mid"));
    SmartDashboard.putData(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_PREP_HIGH_TRAVERSAL_FORWARDS).withName("Climber M Prep High Traversal Forwards"));
    SmartDashboard.putData(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_PREP_HIGH_TRAVERSAL_REVERSE).withName("Climber M Prep High Traversal Reverse"));
    SmartDashboard.putData(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_RAISE_LOW).withName("Climber M Raise Low"));
    SmartDashboard.putData(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_RAISE_MID).withName("Climber M Raise Mid"));
    SmartDashboard.putData(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_RAISE_HIGH_TRAVERSAL_FORWARDS).withName("Climber M Raise High Traversal Forwards"));
    SmartDashboard.putData(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_RAISE_HIGH_TRAVERSAL_REVERSE).withName("Climber M Raise High Traversal Reverse"));
    SmartDashboard.putData(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_CLIMB_LOW).withName("Climber M Climb Low"));
    SmartDashboard.putData(new ClimberStateMachineExecutor(m_climber, m_sensors, ClimberConstants.M_CLIMB_MID).withName("Climber M Climb Mid"));
  }

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(m_arcadeDrive);
    m_intake.setDefaultCommand(m_stowIntake);
    m_centralizer.setDefaultCommand(m_centralizerCargolizer);
    m_chamber.setDefaultCommand(m_chamberCargolizer);

    // m_hood.setDefaultCommand(new RunCommand(m_hood::hoodAuto, m_hood));
    m_hood.setDefaultCommand(new HoodTrackCombo(m_hood, m_targeting));
    // m_shooter.setDefaultCommand(new RunCommand(m_shooter::setFlywheelSpeedAuto, m_shooter));
    m_shooter.setDefaultCommand(new ShooterTrackCombo(m_shooter, m_targeting));
    // m_turret.setDefaultCommand(new TurretTrackLimelight(m_turret, m_sensors.limelight));
    m_turret.setDefaultCommand(new TurretTrackCombo(m_turret, m_targeting));

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
    System.out.println("Auto: " + m_autoCommandName);
    return m_autoCommand;
  }
}
