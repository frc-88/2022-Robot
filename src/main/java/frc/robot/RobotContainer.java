// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.AutoFollowTrajectory;
import frc.robot.commands.drive.DriveDistanceMeters;
import frc.robot.commands.drive.TankDrive;
import frc.robot.commands.feeder.FeederAcceptCargo;
import frc.robot.commands.feeder.FeederCargolizer;
import frc.robot.commands.turret.TurretCalibrate;
import frc.robot.commands.turret.TurretMotionMagicJoystick;
import frc.robot.commands.turret.TurretRawJoystick;
import frc.robot.commands.turret.TurretTrack;
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
  private final Hood m_hood = new Hood(m_sensors);
  private final Shooter m_shooter = new Shooter(m_hood, new CargoSource[]{m_chamber, m_centralizer}, m_sensors);
  private final Climber m_climber = new Climber(m_sensors::isCoastButtonPressed);

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

  private CommandBase m_ingestCargo = new RunCommand(() -> {
        m_intake.deploy();
        m_intake.rollerIntake();
      }, m_intake);

  private CommandBase m_outgestCargo = new ParallelCommandGroup(new RunCommand(() -> {
        m_intake.deploy();
        m_intake.rollerOutgest();
      }, m_intake),
      new RunCommand(m_chamber::reverse, m_chamber),
      new RunCommand(m_centralizer::reverse, m_centralizer));

  private CommandBase m_stowIntake = new RunCommand(() -> {
        m_intake.stow();
        m_intake.rollerStop();
      }, m_intake);

  private CommandBase m_centralizerCargolizer = new FeederCargolizer(m_centralizer, m_intake, m_chamber);
  private CommandBase m_chamberCargolizer = new FeederCargolizer(m_chamber, m_centralizer, m_shooter);

  /////////////////////////////////////
  //            SHOOTING             //
  /////////////////////////////////////

  private CommandBase m_startFlywheel = new RunCommand(m_shooter::setFlywheelSpeedAuto, m_shooter);
  private CommandBase m_stopFlywheel = new InstantCommand(() -> {m_shooter.setFlywheelSpeed(0.0);}, m_shooter);

  private CommandBase m_hoodUp = new RunCommand(m_hood::raiseHood, m_hood);
  private CommandBase m_hoodDown = new RunCommand(m_hood::lowerHood, m_hood);

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

  private CommandBase m_autoCommand = new WaitCommand(1);

  private CommandBase m_oneBallTaxi = new SequentialCommandGroup(
      new TiltCameraDown(m_sensors),
      new ParallelCommandGroup(
        new RunCommand(() -> {m_intake.deploy(); m_intake.rollerIntake();}, m_intake),
        new InstantCommand(() -> {m_shooter.setFlywheelSpeedAuto();}, m_shooter),
        new InstantCommand(m_turret::startTracking),
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new InstantCommand(m_shooter::activate),
          new WaitCommand(5.0),
          new DriveDistanceMeters(m_drive, 1.5, 0.5)
        )
      )
    );

    private CommandBase m_twoBall = new SequentialCommandGroup(
      new TiltCameraDown(m_sensors),
      new ParallelCommandGroup(
        new RunCommand(() -> {m_intake.deploy(); m_intake.rollerIntake();}, m_intake),
        new RunCommand(m_hood::raiseHood, m_hood),
        new InstantCommand(() -> {m_shooter.setFlywheelSpeedAuto();}, m_shooter),
        new InstantCommand(m_turret::startTracking),
        new SequentialCommandGroup(
          new DriveDistanceMeters(m_drive, 1.5, 0.5),
          new WaitCommand(0.5),
          new InstantCommand(m_shooter::activate)
        )
      )
    );

    private CommandBase setupROSAutonomousCommand(int autoIndex)
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
  

  /////////////////////////////////////////////////////////////////////////////
  //                                 SETUP                                   //
  /////////////////////////////////////////////////////////////////////////////

  public RobotContainer(Robot robot) {
    configurePeriodics(robot);
    configureButtonBox();
    configureDefaultCommands();
    configureDashboardCommands();
  }

  private void configurePeriodics(Robot robot) {
    robot.addPeriodic(m_ros_interface::update, 1.0 / 60.0, 0.01);
  }
  
  public void disabledPeriodic() {
    SmartDashboard.putString("Auto", m_autoCommand.toString());

    if (m_buttonBox.isShootButtonPressed()) {
      // m_autoCommand = setupROSAutonomousCommand();
      m_autoCommand = m_oneBallTaxi;
    }
  }

  private void configureButtonBox() {
    m_buttonBox.intakeButton.whileHeld(m_ingestCargo);
    m_buttonBox.outgestButton.whileHeld(m_outgestCargo);

    m_buttonBox.centralizerUp.whileHeld(new RunCommand(m_centralizer::run, m_centralizer));
    m_buttonBox.centralizerDown.whileHeld(new RunCommand(m_centralizer::reverse, m_centralizer));
    m_buttonBox.chamberUp.whileHeld(new RunCommand(m_chamber::run, m_centralizer));
    m_buttonBox.chamberDown.whileHeld(new RunCommand(m_chamber::reverse, m_centralizer));

    m_buttonBox.shootButton.whenPressed(new InstantCommand(m_shooter::activate));
    m_buttonBox.shootButton.whenReleased(new InstantCommand(m_shooter::deactivate));
    m_buttonBox.hoodSwitch.whenPressed(m_hoodUp);
    m_buttonBox.hoodSwitch.whenReleased(m_hoodDown);
    m_buttonBox.flywheelSwitch.whenPressed(m_startFlywheel);
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
  }

  private void configureDashboardCommands() {
    // Drive testing commands
    SmartDashboard.putData("Drive forwards", new ArcadeDrive(m_drive, () -> 5, () -> 0, () -> m_drive.shiftToLow(), () -> Constants.MAX_SPEED_LOW));
    SmartDashboard.putData("Drive Basic Tank", new TankDrive(m_drive, m_testController2::getLeftStickY, m_testController2::getRightStickY));

    // Trajectory testing commands
    SmartDashboard.putData("Ten Feet Forward", new AutoFollowTrajectory(m_drive, m_sensors, RapidReactTrajectories.generateTestTrajectory()));
    // SmartDashboard.putData("Barrel Run", new AutoFollowTrajectory(m_drive, m_sensors, RapidReactTrajectories.generateBarrelRunTrajectory()));
    // SmartDashboard.putData("Barrel Run 2", new AutoFollowTrajectory(m_drive, m_sensors, RapidReactTrajectories.generateBarrelRun2Trajectory()));

    // Intake testing commands
    SmartDashboard.putData("Intake:Ingest", m_ingestCargo);
    SmartDashboard.putData("Intake:Outgest", m_outgestCargo);
    SmartDashboard.putData("Intake:Stow", m_stowIntake);

    // Centralizer and Chamber commmands
    SmartDashboard.putData("Centralizer:AcceptCargo", new FeederAcceptCargo(m_centralizer));
    SmartDashboard.putData("Centralizer:Run", new RunCommand(m_centralizer::run, m_centralizer));
    SmartDashboard.putData("Centralizer:Reverse", new RunCommand(m_centralizer::reverse, m_centralizer));
    SmartDashboard.putData("Centralizer:Stop", new RunCommand(m_centralizer::stop, m_centralizer));
    SmartDashboard.putData("Centralizer:Cargolizer", m_centralizerCargolizer);
    
    SmartDashboard.putData("Chamber:AcceptCargo", new FeederAcceptCargo(m_chamber));
    SmartDashboard.putData("Chamber:Run", new RunCommand(m_chamber::run, m_chamber));
    SmartDashboard.putData("Chamber:Reverse", new RunCommand(m_chamber::reverse, m_chamber));
    SmartDashboard.putData("Chamber:Stop", new RunCommand(m_chamber::stop, m_chamber));
    SmartDashboard.putData("Chamber:Cargolizer", m_chamberCargolizer);

    // Turret test commands
    SmartDashboard.putData("Turret Raw Control",new TurretRawJoystick(m_turret, m_testController2));
    SmartDashboard.putData("Turret Motion Magic Control",new TurretMotionMagicJoystick(m_turret, m_testController2));
    SmartDashboard.putData("Turret Go To 180", new InstantCommand(() -> {m_turret.goToFacing(180);}, m_turret));
    SmartDashboard.putData("Turret Go To 45", new InstantCommand(() -> {m_turret.goToFacing(45);}, m_turret));
    SmartDashboard.putData("Turret Go To 0", new InstantCommand(() -> {m_turret.goToFacing(0);}, m_turret));
    SmartDashboard.putData("Turret Go To -45", new InstantCommand(() -> {m_turret.goToFacing(-45);}, m_turret));
    SmartDashboard.putData("Turret Go To -180", new InstantCommand(() -> {m_turret.goToFacing(-180);}, m_turret));
    
    SmartDashboard.putData("Turret Track", new TurretTrack(m_turret, m_sensors.limelight));
    SmartDashboard.putData("Turret Activate Tracking", new InstantCommand(m_turret::startTracking));
    SmartDashboard.putData("Turret Deactivate Tracking", new InstantCommand(m_turret::stopTracking));
    SmartDashboard.putData("Turret !!Calibrate!!", new TurretCalibrate(m_turret));
    SmartDashboard.putData("Turret Sync", new InstantCommand(m_turret::sync, m_turret));

    // Shooter testing commands
    SmartDashboard.putData("Shooter:Flywheel:RunRaw", new InstantCommand(() -> {m_shooter.setFlywheelRaw(new DoublePreferenceConstant("Shooter Test Output", 0.0).getValue());}, m_shooter));
    SmartDashboard.putData("Shooter:Flywheel:StopRaw", new InstantCommand(() -> {m_shooter.setFlywheelRaw(0.0);}, m_shooter));
    SmartDashboard.putData("Shooter:Flywheel:RunSpeed", new InstantCommand(() -> {m_shooter.setFlywheelSpeed(new DoublePreferenceConstant("Shooter Test Speed", 0.0).getValue());}, m_shooter));
    SmartDashboard.putData("Shooter:Flywheel:RunAuto", m_startFlywheel);
    SmartDashboard.putData("Shooter:Flywheel:StopSpeed", m_stopFlywheel);
    SmartDashboard.putData("Shooter:Activate", new InstantCommand(m_shooter::activate, m_shooter));
    SmartDashboard.putData("Shooter:Deactivate", new InstantCommand(m_shooter::deactivate, m_shooter));

    SmartDashboard.putData("Hood:Raise", m_hoodUp);
    SmartDashboard.putData("Hood:Lower", m_hoodDown);
    SmartDashboard.putData("Hood:UpRaw", new RunCommand(()->{m_hood.setHoodPercentOut(1);}, m_hood));
    SmartDashboard.putData("Hood:DownRaw", new RunCommand(()->{m_hood.setHoodPercentOut(-1);}, m_hood));
    SmartDashboard.putData("Hood:StopRaw", new RunCommand(()->{m_hood.setHoodPercentOut(0);}, m_hood));

    // Limelight
    SmartDashboard.putData("Limelight On", new LimelightToggle(m_sensors.limelight, true));
    SmartDashboard.putData("Limelight Off", new LimelightToggle(m_sensors.limelight, false));

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
    m_centralizer.setDefaultCommand(m_centralizerCargolizer);
    m_chamber.setDefaultCommand(m_chamberCargolizer);

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
