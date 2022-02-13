// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sensors;
import frc.robot.util.tunnel.ROSInterface;
import frc.robot.util.tunnel.TunnelServer;
import frc.robot.subsystems.Turret;
import frc.robot.util.RapidReactTrajectories;
import frc.robot.util.controllers.ButtonBox;
import frc.robot.util.controllers.DriverController;
import frc.robot.util.controllers.FrskyDriverController;
import frc.robot.util.controllers.XboxController;
import frc.robot.util.controllers.ButtonBox.ClimbAction;
import frc.robot.util.controllers.ButtonBox.ClimbBar;
import frc.robot.util.controllers.ButtonBox.ClimbDirection;
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
  private final Intake m_intake = new Intake();
  private final Turret m_turret = new Turret();
  private final Feeder m_centralizer = new Feeder(Constants.FEEDER_CENTRALIZER_MOTOR_ID, Constants.FEEDER_CENTRALIZER_BEAMBREAK, new DoublePreferenceConstant("Centralizer:Speed", Constants.FEEDER_CENTRALIZER_SPEED_DFT));
  private final Feeder m_chamber = new Feeder(Constants.FEEDER_CHAMBER_MOTOR_ID, Constants.FEEDER_CHAMBER_BEAMBREAK, new DoublePreferenceConstant("Chamber:Speed",Constants.FEEDER_CHAMBER_SPEED_DFT));
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
  private ROSInterface m_ros_interface = new ROSInterface(m_drive);
  private TunnelServer m_tunnel = new TunnelServer(m_ros_interface, 5800, 15);



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
  private CommandBase m_outgestCargo = new RunCommand(() -> {
        m_intake.deploy();
        m_intake.rollerOutgest();
      }, m_intake);;
  private CommandBase m_stowIntake = new RunCommand(() -> {
        m_intake.stow();
        m_intake.rollerStop();
      }, m_intake);;

  private CommandBase m_shoot = new WaitCommand(1);

  private CommandBase m_turretTrackingMode = new InstantCommand(m_turret::startTracking);

  private CommandBase m_stowShooter = new InstantCommand(m_turret::stopTracking);

  /////////////////////////////////////
  //             CLIMBER             //
  /////////////////////////////////////

  private CommandBase m_calibrateClimber = 
      new RunCommand(m_climber::calibrate, m_climber)
          .withInterrupt(m_climber::isCalibrated)
          .beforeStarting(m_climber::resetCalibration)
          .withName("calibrateClimber");

  private CommandBase m_stowClimber = new WaitCommand(1);
  
  private static class DirectionCommands {
    private CommandBase m_prepCommand;
    private CommandBase m_raiseCommand;
    private CommandBase m_climbCommand;
    
    public DirectionCommands(CommandBase prepCommand, CommandBase raiseCommand, CommandBase climbCommand) {
      m_prepCommand = prepCommand;
      m_raiseCommand = raiseCommand;
      m_climbCommand = climbCommand;
    }

    public CommandBase getCommand(ClimbAction action) {
      switch (action) {
        case PREP:
          return m_prepCommand;
        case RAISE:
          return m_raiseCommand;
        case CLIMB:
          return m_climbCommand;
        default:
          return new WaitCommand(0);
      }
    }
  }
  public static class BarCommands {

    private DirectionCommands m_forwards;
    private DirectionCommands m_backwards;

    public BarCommands(DirectionCommands forwards, DirectionCommands backwards) {
      m_forwards = forwards;
      m_backwards = backwards;
    }

    public CommandBase getCommand(ClimbDirection direction, ClimbAction action) {
      switch (direction) {
        case FORWARDS:
          return m_forwards.getCommand(action);
        case BACKWARDS:
          return m_backwards.getCommand(action);
        default:
          return new WaitCommand(0);
      }
    }
  }
  private static class ClimbCommands {

    private BarCommands m_low;
    private BarCommands m_mid;
    private BarCommands m_high;
    private BarCommands m_traversal;

    public ClimbCommands(BarCommands low, BarCommands mid, BarCommands high, BarCommands traversal) {
      m_low = low;
      m_mid = mid;
      m_high = high;
      m_traversal = traversal;
    }

    public CommandBase getCommand(ClimbBar bar, ClimbDirection direction, ClimbAction action) {
      switch (bar) {
        case LOW:
          return m_low.getCommand(direction, action);
        case MID:
          return m_mid.getCommand(direction, action);
        case HIGH:
          return m_high.getCommand(direction, action);
        case TRAVERSAL:
          return m_traversal.getCommand(direction, action);
        default:
          return new WaitCommand(0);
      }
    }
  }
  private ClimbCommands m_climbCommands = new ClimbCommands(
    new BarCommands(
      new DirectionCommands(
        new WaitCommand(1), // low, forwards, prep
        new WaitCommand(1), // low, forwards, raise
        new WaitCommand(1) // low, forwards, climb
      ),
      new DirectionCommands(
        new WaitCommand(1), // low, backwards, prep
        new WaitCommand(1), // low, backwards, raise
        new WaitCommand(1) // low, backwards, climb
      )
    ), new BarCommands(
      new DirectionCommands(
        new WaitCommand(1), // mid, forwards, prep
        new WaitCommand(1), // mid, forwards, raise
        new WaitCommand(1) // mid, forwards, climb
      ),
      new DirectionCommands(
        new WaitCommand(1), // mid, backwards, prep
        new WaitCommand(1), // mid, backwards, raise
        new WaitCommand(1) // mid, backwards, climb
      )
    ), new BarCommands(
      new DirectionCommands(
        new WaitCommand(1), // high, forwards, prep
        new WaitCommand(1), // high, forwards, raise
        new WaitCommand(1) // high, forwards, climb
      ),
      new DirectionCommands(
        new WaitCommand(1), // high, backwards, prep
        new WaitCommand(1), // high, backwards, raise
        new WaitCommand(1) // high, backwards, climb
      )
    ), new BarCommands(
      new DirectionCommands(
        new WaitCommand(1), // traversal, forwards, prep
        new WaitCommand(1), // traversal, forwards, raise
        new WaitCommand(1) // traversal, forwards, climb
      ),
      new DirectionCommands(
        new WaitCommand(1), // traversal, backwards, prep
        new WaitCommand(1), // traversal, backwards, raise
        new WaitCommand(1) // traversal, backwards, climb
      )
    ));

  private CommandBase m_manualModeClimber = new ManualModeClimber(m_climber, m_testController);
  private CommandBase m_climberTestMotionMagic = new ClimberTestMotionMagic(m_climber);;
  private CommandBase m_climberMotionMagicJoystick = new ClimberMotionMagicJoystick(m_climber, m_testController);

  /////////////////////////////////////
  //              AUTO               //
  /////////////////////////////////////

  private final CommandBase m_autoCommand = new WaitCommand(15.0);



  /////////////////////////////////////////////////////////////////////////////
  //                                 SETUP                                   //
  /////////////////////////////////////////////////////////////////////////////

  public RobotContainer() {
    configureButtonBox();
    configureDefaultCommands();
    configureDashboardCommands();
  }

  private void configureButtonBox() {
    
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
