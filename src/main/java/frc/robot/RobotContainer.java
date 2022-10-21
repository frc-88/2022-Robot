// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.Trajectory;
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
import frc.robot.commands.test.CollectLimelightRectification;
import frc.robot.commands.turret.ShooterTrackCombo;
import frc.robot.commands.turret.TurretCalibrate;
import frc.robot.commands.turret.TurretLock;
import frc.robot.commands.turret.TurretMotionMagicJoystick;
import frc.robot.commands.turret.TurretRawJoystick;
import frc.robot.commands.turret.TurretTrackLimelight;
import frc.robot.commands.turret.TurretTrackCombo;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Targeting;
import frc.robot.subsystems.Turret;
import frc.robot.util.CargoSource;
import frc.robot.util.NumberCache;
import frc.robot.util.RapidReactTrajectories;
// import frc.robot.util.RapidReactTrajectories;
import frc.robot.util.climber.ClimberConstants;
import frc.robot.util.controllers.ButtonBox;
import frc.robot.util.controllers.DriverController;
import frc.robot.util.controllers.FrskyController;
import frc.robot.util.controllers.FrskyDriverController;
import frc.robot.util.controllers.XboxController;
import frc.robot.util.controllers.ButtonBox.ClimbBar;
import frc.robot.util.controllers.ButtonBox.ClimbDirection;
import frc.robot.Constants.AutoConstants;
import frc.robot.util.coprocessor.networktables.SwerveTable;
import frc.robot.commands.LimelightToggle;
import frc.robot.commands.ShootAll;
// import frc.robot.commands.autos.AutoFollowTrajectory;
// import frc.robot.commands.autos.DriveToCargo;
// import frc.robot.commands.autos.DriveToWaypoint;
// import frc.robot.commands.autos.DriveToWaypointWithHeading;
// import frc.robot.commands.autos.DriveWithWaypointsPlan;
// import frc.robot.commands.autos.PassthroughRosCommand;
// import frc.robot.commands.autos.SetGlobalPoseToLimelight;
// import frc.robot.commands.autos.SetGlobalPoseToWaypoint;
import frc.robot.commands.cameratilter.TiltCameraDown;
import frc.robot.commands.climber.ClimberMotionMagicJoystick;
import frc.robot.commands.climber.ClimberStateMachineExecutor;
import frc.robot.commands.climber.ClimberTestMotionMagic;
import frc.robot.commands.climber.ManualModeClimber;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.drive.GrantDriveCommand;
// import frc.robot.commands.drive.DriveDegrees;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PreferenceConstants;

public class RobotContainer {
  /////////////////////////////////////////////////////////////////////////////
  //                              SUBSYSTEMS                                 //
  /////////////////////////////////////////////////////////////////////////////
  private final Sensors m_sensors = new Sensors();
  private final SwerveDrive m_drive = new SwerveDrive(m_sensors.ahrs_navx);
  private final Climber m_climber = new Climber(m_sensors::isCoastButtonPressed);
  private final Intake m_intake = new Intake();
  private final Turret m_turret = new Turret(m_sensors);
  private final Feeder m_feeder = new Feeder();
  private final SwerveTable m_ros_interface = new SwerveTable(
    m_drive,
    Robot.isSimulation() ? Constants.COPROCESSOR_ADDRESS_SIMULATED : Constants.COPROCESSOR_ADDRESS,
    Constants.COPROCESSOR_PORT,
    Constants.COPROCESSOR_TABLE_UPDATE_DELAY,
    m_climber.outerArm, m_climber.innerArm, m_intake, m_turret, m_sensors
  );
  private final Shooter m_shooter = new Shooter(m_sensors, m_drive, m_turret, m_feeder, m_ros_interface);

  private final Navigation m_nav = new Navigation(m_ros_interface);
  private final Targeting m_targeting = new Targeting(m_sensors.limelight, m_ros_interface, m_turret, m_drive, m_shooter);

  /////////////////////////////////////////////////////////////////////////////
  //                              CONTROLLERS                                //
  /////////////////////////////////////////////////////////////////////////////
  private final DriverController m_driverController = new FrskyDriverController(Constants.DRIVER_CONTROLLER_ID);
  private final ButtonBox m_buttonBox = new ButtonBox(Constants.BUTTON_BOX_ID);
  private final XboxController m_testController = new XboxController(Constants.TEST_CONTROLLER_ID);
  private final XboxController m_testController2 = new XboxController(Constants.TEST_CONTROLLER_2_ID);
  
  private DoublePreferenceConstant p_shooterAutoSpeed = new DoublePreferenceConstant("Shooter Auto Speed", 0.0);
  private final SlewRateLimiter filterX = new SlewRateLimiter(4.0);
  private final SlewRateLimiter filterY = new SlewRateLimiter(4.0);

  private boolean m_hasConfiguredDashboardButtons = false;

  /////////////////////////////////////////////////////////////////////////////
  //                               COMMANDS                                  //
  /////////////////////////////////////////////////////////////////////////////

  /////////////////////////////////////
  //             DRIVE               //
  /////////////////////////////////////

  // Set up the default command for the drivetrain.
  // The controls are for field-oriented driving:
  // Left stick Y axis -> forward and backwards movement
  // Left stick X axis -> left and right movement
  // Right stick X axis -> rotation
  private CommandBase m_swerveDrive =    
    new SwerveDriveCommand(
      m_drive,
      () -> modifyAxis(filterY.calculate(m_driverController.getTranslationY())) * SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND,
      () -> modifyAxis(filterX.calculate(m_driverController.getTranslationX())) * SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND,
      () -> modifyAxis(m_driverController.getRotation()) * SwerveDrive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    );
    
    private CommandBase m_grantDrive =    
    new GrantDriveCommand(
      m_drive,
      () -> modifyAxis(filterY.calculate(((FrskyController) m_driverController).getLeftStickY())) * SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND,
      () -> modifyAxis(((FrskyController) m_driverController).getRightStickX()),
      () -> modifyAxis(((FrskyController) m_driverController).getRightStickY()),
      () -> -modifyAxis(((FrskyController) m_driverController).getLeftStickX()) * SwerveDrive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    );

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
          } else {
            return (value + deadband) / (1.0 - deadband);
          }
        } else {
          return 0.0;
        }
      }
    
      private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);
    
        // Square the axis
        value = Math.copySign(value * value, value);
    
        return value;
      }



  /////////////////////////////////////
  //          BALL HANDLING          //
  /////////////////////////////////////

  private CommandBase m_ingestCargo = new RunCommand(() -> {
        m_intake.deploy();
        m_intake.rollerIntake();
      }, m_intake);

  private CommandBase m_outgestCargo = new RunCommand(() -> {
        m_intake.deploy();
        m_intake.rollerOutgest();
      }, m_intake);

  private CommandBase m_stowIntake = new RunCommand(() -> {
        m_intake.stow();
        m_intake.rollerStop();
      }, m_intake);


  /////////////////////////////////////
  //            SHOOTING             //
  /////////////////////////////////////

  private CommandBase m_startFlywheel = new ShooterTrackCombo(m_shooter, m_targeting);
  private CommandBase m_stopFlywheel = new InstantCommand(() -> {m_shooter.setFlywheelSpeed(0.0);}, m_shooter);
  private CommandBase m_flywheelFenderShot = new RunCommand(m_shooter::setFlywheelFenderShot, m_shooter);

  private boolean m_hoodCalibrated = false;

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
  
  private ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
  private Trajectory testTrajectory = RapidReactTrajectories.generatePathWeaverTrajectory("swervetest.wpilib.json");

SwerveControllerCommand swerveControllerCommand =
  new SwerveControllerCommand(
      testTrajectory,
      m_drive::getOdometryPose, // Functional interface to feed supplier
      m_drive.kinematics,
      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      thetaController,
      m_drive::setModuleStates,
      m_drive);

  private CommandBase m_autoTwoBall = 
  new ParallelCommandGroup(
    new TiltCameraDown(m_sensors),
    new InstantCommand(m_turret::startTracking),
    new InstantCommand(m_sensors.limelight::ledOn),
    new InstantCommand(() -> m_turret.setDefaultFacing(0)),
    new InstantCommand(() -> m_targeting.setModeToLimelight()),
    new RunCommand(() -> {m_intake.deploy(); m_intake.rollerIntake();}, m_intake),
    new SequentialCommandGroup(
      new WaitCommand(0.5),
      new ShootAll(m_shooter),
      new FollowTrajectory(m_drive, RapidReactTrajectories.generatePathWeaverTrajectory("SwerveTwo.wpilib.json"), true),
      new WaitCommand(0.5)
    )
  );

  private CommandBase m_autoThreeBall = 
  new ParallelCommandGroup(
    new TiltCameraDown(m_sensors),
    new InstantCommand(m_turret::startTracking),
    new InstantCommand(m_sensors.limelight::ledOn),
    new InstantCommand(() -> m_turret.setDefaultFacing(0)),
    new InstantCommand(() -> m_targeting.setModeToLimelight()),
    new RunCommand(() -> {m_intake.deploy(); m_intake.rollerIntake();}, m_intake),
    new SequentialCommandGroup(
      new WaitCommand(0.5),
      new ShootAll(m_shooter),
      new FollowTrajectory(m_drive, RapidReactTrajectories.generatePathWeaverTrajectory("SwerveThree.wpilib.json"), true),
      new WaitCommand(0.5),
      new ShootAll(m_shooter)
    )
  );

  private CommandBase m_autoSixBall = 
    new ParallelCommandGroup(
      new TiltCameraDown(m_sensors),
      new InstantCommand(m_turret::startTracking),
      new InstantCommand(m_sensors.limelight::ledOn),
      new InstantCommand(() -> m_turret.setDefaultFacing(0)),
      new InstantCommand(() -> m_targeting.setModeToLimelight()),
      new RunCommand(() -> {m_intake.deploy(); m_intake.rollerIntake();}, m_intake),
      new ShooterTrackCombo(m_shooter, m_targeting),
      new SequentialCommandGroup(
        // new InstantCommand(() -> m_targeting.disableDefault()),
        // new InstantCommand(() -> m_targeting.enableDefault(97, -14.5)),
        new WaitCommand(0.5),
        new ShootAll(m_shooter).withTimeout(3.0),
        new FollowTrajectory(m_drive, RapidReactTrajectories.generatePathWeaverTrajectory("SwerveSix.wpilib.json"), true),
        // new InstantCommand(() -> m_targeting.enableDefault(159, 30)),
        // new InstantCommand(m_targeting::disableDefault),
        new WaitCommand(0.5),
        new ShootAll(m_shooter).withTimeout(4.0),
        new FollowTrajectory(m_drive, RapidReactTrajectories.generatePathWeaverTrajectory("SwerveSix_0.wpilib.json"), false),
        new ShootAll(m_shooter)
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

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    configurePeriodics(robot);
    configureButtonBox();
    configureDefaultCommands();

    m_drive.resetOdometry(testTrajectory.sample(0).poseMeters, m_drive.getGyroscopeRotation());
  }

  private void configurePeriodics(Robot robot) {
    // robot.addPeriodic(m_ros_interface::update, Constants.COPROCESSOR_PERIODIC_UPDATE_DELAY, Constants.COPROCESSOR_PERIODIC_UPDATE_OFFSET);
    // robot.addPeriodic(m_ros_interface::updateSlow, Constants.COPROCESSOR_SLOW_PERIODIC_UPDATE_DELAY, Constants.COPROCESSOR_SLOW_PERIODIC_UPDATE_OFFSET);

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
    if (m_buttonBox.isShootButtonPressed() && !m_autoCommandName.equals("6 Cargo")) {
      m_autoCommand = m_autoSixBall;
      m_autoCommandName = "6 Cargo";
      // new SetGlobalPoseToWaypoint(m_nav, "<team>_start_5").schedule();

    }

    if (m_buttonBox.isChamberUpButtonPressed() && !m_autoCommandName.equals("3 Cargo")) {
      m_autoCommand = m_autoThreeBall;
      m_autoCommandName = "3 Cargo";
      // new SetGlobalPoseToWaypoint(m_nav, "<team>_start_2").schedule();
    }

    if (m_buttonBox.isChamberDownButtonPressed() && !m_autoCommandName.equals("2 Cargo")) {
      m_autoCommand = m_autoTwoBall;
      m_autoCommandName = "2 Cargo";
      // new SetGlobalPoseToWaypoint(m_nav, "<team>_start_2").schedule();
    }

    if (m_buttonBox.isCentralizerUpButtonPressed() && !m_autoCommandName.equals("Wait 1")) {
      m_autoCommand = new WaitCommand(1.0);
      m_autoCommandName = "Wait 1";
      // new SetGlobalPoseToWaypoint(m_nav, "center").schedule();
    }

    // if (m_buttonBox.isDefaultTurretButtonPressed() && !m_autoCommandName.equals("1 Ball Left")) {
    //   m_autoCommand = m_autoOneBallLeft;
    //   m_autoCommandName = "1 Ball Left";
    //   new SetGlobalPoseToWaypoint(m_nav, "<team>_start_1L").schedule();
    // }

    // if (m_buttonBox.isCancelClimbPressed() && !m_autoCommandName.equals("1 Ball Left")) {
    //   m_autoCommand = m_autoOneBallRight;
    //   m_autoCommandName = "1 Ball Right";
    //   new SetGlobalPoseToWaypoint(m_nav, "<team>_start_1R").schedule();
    // }

    // if (m_buttonBox.isOutgestButtonPressed() && !m_autoCommandName.equals("Pursuit Only")) {
    //   m_autoCommand = m_autoPursuitOnly;
    //   m_autoCommandName = "Pursuit Only";
    // }

    if (m_buttonBox.isCentralizerDownButtonPressed()) {
      m_sensors.limelight.ledOn();
    } else {
      m_sensors.limelight.ledOff();
    }

    SmartDashboard.putString("Auto", m_autoCommandName);
  }

  public void teleopInit() {
    if (!m_hoodCalibrated) {
      m_shooter.calibrateHood();
      m_hoodCalibrated = true;
    }

    if (m_buttonBox.isAutoShootSwitchOn()) {
      m_shooter.activateRestrictive();
    } else {
      m_shooter.deactivate();
    }
    
    if (m_buttonBox.isTrackTurretSwitchOn()) {
      m_turret.startTracking();
      m_startFlywheel.schedule();
      
    } else {
      m_turret.stopTracking();
      m_flywheelFenderShot.schedule();
    }

    m_turret.setDefaultFacing(0.);
    m_targeting.disableDefault();
    // m_ros_interface.enableCargoMarauding();
    // m_ros_interface.disableCargoMarauding();
  }

  public void autoInit() {
    if (!m_hoodCalibrated) {
      m_shooter.calibrateHood();
      m_hoodCalibrated = true;
    }
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
    new Button(m_driverController::getGyroReset).whenPressed(new InstantCommand(m_drive::zeroGyroscope));

    m_buttonBox.intakeButton.whileHeld(m_ingestCargo);
    m_buttonBox.outgestButton.whileHeld(m_outgestCargo);

    m_buttonBox.shootButton.whenPressed(new InstantCommand(m_shooter::activatePermissive));
    m_buttonBox.shootButton.whenReleased(new ConditionalCommand(new InstantCommand(m_shooter::activateRestrictive), new InstantCommand(m_shooter::deactivate), m_buttonBox::isAutoShootSwitchOn));
    m_buttonBox.autoShootSwitch.whenPressed(new InstantCommand(m_shooter::activateRestrictive));
    m_buttonBox.autoShootSwitch.whenReleased(new ConditionalCommand(new InstantCommand(m_shooter::activatePermissive), new InstantCommand(m_shooter::deactivate), m_buttonBox::isShootButtonPressed));


    m_buttonBox.turretTrackSwitch.whenPressed(new InstantCommand(m_turret::startTracking));
    m_buttonBox.turretTrackSwitch.whenPressed(m_startFlywheel);
    m_buttonBox.turretTrackSwitch.whenReleased(new InstantCommand(m_turret::stopTracking));
    m_buttonBox.turretTrackSwitch.whenReleased(m_flywheelFenderShot);
    
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
      new RunCommand(() -> { m_shooter.setFlywheelSpeed(0); m_shooter.hoodDown(); }, m_shooter),
      new SequentialCommandGroup(
        new WaitUntilCommand(m_turret::isSafeForClimber),
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
      new RunCommand(() -> { m_shooter.setFlywheelSpeed(0); m_shooter.hoodDown(); }, m_shooter),
      new SequentialCommandGroup(
        new WaitUntilCommand(m_turret::isSafeForClimber),
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
      new RunCommand(() -> { m_shooter.setFlywheelSpeed(0); m_shooter.hoodDown(); }, m_shooter),
      new SequentialCommandGroup(
        new WaitUntilCommand(m_turret::isSafeForClimber),
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
    // Autonomous testing
    // SmartDashboard.putData("Auto Two Ball", m_autoTwoBall);
    // SmartDashboard.putData("Auto Five Ball", m_autoFiveBall);
    SmartDashboard.putData("Tilt Camera Down", new TiltCameraDown(m_sensors));

    // Trajectory testing
    // SmartDashboard.putData("Ten Feet Trajectory", new AutoFollowTrajectory(m_drive, RapidReactTrajectories.generateStraightTrajectory(10.0), true));
    // SmartDashboard.putData("Five Ball Trajectory", new AutoFollowTrajectory(m_drive, RapidReactTrajectories.generatePathWeaverTrajectory("ThreeForThreeInThree.wpilib.json"), true));
    SmartDashboard.putData("Swerve Controller Test", swerveControllerCommand);
    SmartDashboard.putData("Swerve Test Trajectory", new FollowTrajectory(m_drive, testTrajectory, true));
    SmartDashboard.putData("Swerve Five", new FollowTrajectory(m_drive, RapidReactTrajectories.generatePathWeaverTrajectory("swerve5_leg1.wpilib.json"), true));
    SmartDashboard.putData("Zero Drive", new InstantCommand(() -> {m_drive.zeroGyroscope();}));

    // Intake testing commands
    SmartDashboard.putData("Intake:Ingest", m_ingestCargo);
    SmartDashboard.putData("Intake:Outgest", m_outgestCargo);
    SmartDashboard.putData("Intake:Stow", m_stowIntake);

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
    // m_drive.setDefaultCommand(m_swerveDrive);
     m_drive.setDefaultCommand(m_grantDrive);
    m_intake.setDefaultCommand(m_stowIntake);
    m_feeder.setDefaultCommand(new RunCommand(() -> {
      if (m_shooter.wantsCargo()) {
        m_feeder.shoot();
      } else if (m_intake.isIntaking()) {
        m_feeder.intake();
      } else {
        m_feeder.hold();
      }
    }, m_feeder));

    // m_hood.setDefaultCommand(new RunCommand(m_hood::hoodAuto, m_hood));
    // m_hood.setDefaultCommand(new HoodTrackCombo(m_hood, m_targeting));
    // m_shooter.setDefaultCommand(new RunCommand(m_shooter::setFlywheelSpeedAuto, m_shooter));
    // m_shooter.setDefaultCommand(new ShooterTrackCombo(m_shooter, m_targeting));
    m_turret.setDefaultCommand(new TurretTrackLimelight(m_turret, m_sensors.limelight));
    // m_turret.setDefaultCommand(new TurretTrackCombo(m_turret, m_targeting));
    // m_turret.setDefaultCommand(new TurretLock(m_turret));

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
