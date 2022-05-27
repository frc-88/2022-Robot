// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.CargoSource;
import frc.robot.util.CargoTarget;
import frc.robot.util.ThisRobotTable;
import frc.robot.util.ValueInterpolator;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

/**
 * Cargo is coming
 * Is it blue or is it red?
 * Mine scores, yours bounces
 **/

public class Shooter extends SubsystemBase implements CargoTarget {
  private TalonFX m_flywheel = new TalonFX(Constants.SHOOTER_FLYWHEEL_ID, "1");
  private CargoSource[] m_sources;
  private Sensors m_sensors;
  private Hood m_hood;
  private Turret m_turret;
  private Drive m_drive;
  private ThisRobotTable m_ros_interface;
  private DoubleSupplier m_shotProbabilitySupplier;

  private static enum ActiveMode {
    DEACTIVATED,
    ACTIVE_PERMISSIVE,
    ACTIVE_RESTRICTIVE
  }

  private ActiveMode m_active = ActiveMode.DEACTIVATED;
  private Timer m_cargoWaitTimer = new Timer();
  private boolean m_cargoWaiting = false;
  private boolean m_wantedCargo = false;
  private Timer m_restrictiveCheckTimer = new Timer();

  private double m_limelightDistance = 0;
  private double m_limelightAngle = 0;
  private double m_rosDistance = 0;
  private double m_rosAngle = 0;

  private static final double FLYWHEEL_RATIO = 1;

  private final ValueInterpolator hoodDownInterpolator = new ValueInterpolator(
      new ValueInterpolator.ValuePair(77, 2200),
      new ValueInterpolator.ValuePair(93, 2250),
      new ValueInterpolator.ValuePair(99, 2300),
      new ValueInterpolator.ValuePair(114, 2375),
      new ValueInterpolator.ValuePair(122, 2450),
      new ValueInterpolator.ValuePair(160, 2450));

  private final ValueInterpolator hoodMidInterpolator = new ValueInterpolator(
      new ValueInterpolator.ValuePair(85.5, 2200),
      new ValueInterpolator.ValuePair(102, 2200),
      new ValueInterpolator.ValuePair(117, 2300),
      new ValueInterpolator.ValuePair(134, 2425),
      new ValueInterpolator.ValuePair(152, 2850),
      new ValueInterpolator.ValuePair(166, 3150));

  private final ValueInterpolator hoodUpInterpolator = new ValueInterpolator(
      new ValueInterpolator.ValuePair(125, 2425),
      new ValueInterpolator.ValuePair(146, 2600),
      new ValueInterpolator.ValuePair(160, 2680),
      new ValueInterpolator.ValuePair(184, 2950),
      new ValueInterpolator.ValuePair(213, 3400),
      new ValueInterpolator.ValuePair(240, 3625),
      new ValueInterpolator.ValuePair(272, 4100),
      new ValueInterpolator.ValuePair(298, 4675));

  // Preferences
  private PIDPreferenceConstants p_flywheelPID = new PIDPreferenceConstants("Shooter PID", 0.0, 0.0, 0.0, 0.047, 0.0,
      0.0, 0.0);
  private DoublePreferenceConstant p_flywheelIdle = new DoublePreferenceConstant("Shooter Idle Speed", 1300.0);
  private DoublePreferenceConstant p_flywheelFenderShotLow = new DoublePreferenceConstant("Shooter Fender Shot Low", 1500.0);
  private DoublePreferenceConstant p_flywheelFenderShotHigh = new DoublePreferenceConstant("Shooter Fender Shot High", 2400.0);
  private DoublePreferenceConstant p_flywheelBlindUp = new DoublePreferenceConstant("Shooter Blind Up Speed", 5000.0);
  private DoublePreferenceConstant p_flywheelBlindDown = new DoublePreferenceConstant("Shooter Blind Down Speed",
      5000.0);
  private DoublePreferenceConstant p_shooterReady = new DoublePreferenceConstant("Shooter Pause (s)", 0.5);
  private DoublePreferenceConstant p_cargoInShooter = new DoublePreferenceConstant("Cargo In Shooter (s)", 0.2);
  private DoublePreferenceConstant p_shooterSpinLimit = new DoublePreferenceConstant("Shooter Spin Limit", 90.);
  private DoublePreferenceConstant p_shooterAccelerationLimit = new DoublePreferenceConstant("Shooter Acceleration Limit", 4.);
  private DoublePreferenceConstant p_shooterProbabilityLimit = new DoublePreferenceConstant("Shooter Probability Limit", 0.4);
  private DoublePreferenceConstant p_restrictiveCheckTime = new DoublePreferenceConstant("Shooter Restrictive Check Time", 0.2);

  /** Creates a new Shooter. */
  public Shooter(Sensors sensors, Hood hood, Drive drive, Turret turret, CargoSource[] sources, ThisRobotTable ros_interface) {
    m_sensors = sensors;
    m_hood = hood;
    m_drive = drive;
    m_turret = turret;
    m_sources = sources;
    m_ros_interface = ros_interface;
    m_shotProbabilitySupplier = () -> 1.;

    m_restrictiveCheckTimer.reset();
    m_restrictiveCheckTimer.start();

    configureFlywheel();

    p_flywheelPID.addChangeHandler((Double unused) -> configureFlywheel());
  }

  private void configureFlywheel() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    config.slot0.kP = p_flywheelPID.getKP().getValue();
    config.slot0.kI = p_flywheelPID.getKI().getValue();
    config.slot0.kD = p_flywheelPID.getKD().getValue();
    config.slot0.kF = p_flywheelPID.getKF().getValue();
    config.slot0.integralZone = p_flywheelPID.getIZone().getValue();
    config.slot0.maxIntegralAccumulator = p_flywheelPID.getIMax().getValue();
    config.neutralDeadband = 0.001;
    config.peakOutputForward = 1.0;
    config.peakOutputReverse = -1.0;
    config.nominalOutputForward = 0.02;
    config.nominalOutputReverse = -0.02;
    m_flywheel.configAllSettings(config);
  }

  public void setFlywheelSpeed(double speed) {
    m_flywheel.set(TalonFXControlMode.Velocity, convertRPMsToMotorTicks(speed));
  }

  public void setFlywheelRaw(double percentOutput) {
    m_flywheel.set(TalonFXControlMode.PercentOutput, percentOutput);
  }

  public void setFlywheelSpeedAuto(double target_dist) {
    if (!m_turret.isTracking() && Math.abs(m_turret.getDefaultFacing()) < 90.) {
      m_flywheel.set(TalonFXControlMode.Velocity, convertRPMsToMotorTicks(p_flywheelFenderShotLow.getValue()));
    } else if (!m_turret.isTracking()) {
      m_flywheel.set(TalonFXControlMode.Velocity, convertRPMsToMotorTicks(p_flywheelFenderShotHigh.getValue()));
    } else {
      m_flywheel.set(TalonFXControlMode.Velocity, convertRPMsToMotorTicks(calcSpeedFromDistance(target_dist)));
    }
  }

  public void setFlywheelFenderShot() {
    if (Math.abs(m_turret.getDefaultFacing()) < 90.) {
      m_flywheel.set(TalonFXControlMode.Velocity, convertRPMsToMotorTicks(p_flywheelFenderShotLow.getValue()));
    } else {
      m_flywheel.set(TalonFXControlMode.Velocity, convertRPMsToMotorTicks(p_flywheelFenderShotHigh.getValue()));
    }
  }

  private double calcSpeedFromDistance(double target_dist) {
    if (target_dist > 0.0) {
      if (m_hood.isDown()) {
        return hoodDownInterpolator.getInterpolatedValue(target_dist);
      } else if (m_hood.isUp()) {
        return hoodUpInterpolator.getInterpolatedValue(target_dist);
      } else {
        return hoodMidInterpolator.getInterpolatedValue(target_dist);
      }
    } else {
      if (m_hood.isDown()) {
        return p_flywheelBlindDown.getValue();
      } else {
        return p_flywheelBlindUp.getValue();
      }
    }
  }

  public boolean onTarget() {
    return Math.abs(convertMotorTicksToRPM(m_flywheel.getClosedLoopError())) < p_flywheelPID.getTolerance().getValue();
  }

  public void activatePermissive() {
    m_active = ActiveMode.ACTIVE_PERMISSIVE;
  }

  public void activateRestrictive() {
    m_active = ActiveMode.ACTIVE_RESTRICTIVE;
  }

  public void deactivate() {
    m_active = ActiveMode.DEACTIVATED;
  }

  private double convertMotorTicksToRPM(double motorVelocity) {
    return motorVelocity * 600 / (FLYWHEEL_RATIO * 2048);
  }

  private double convertRPMsToMotorTicks(double flywheelVelocity) {
    return flywheelVelocity * FLYWHEEL_RATIO * 2048 / 600;
  }

  public boolean sourcesHaveCargo() {
    boolean hasCargo = false;

    for (CargoSource source : m_sources) {
      hasCargo = hasCargo || source.hasCargo();
    }

    return hasCargo;
  }

  public boolean cargoChambered() {
    return m_sources[0].hasCargo();
  }

  public boolean cargoCentralized() {
    return m_sources[1].hasCargo();
  }

  @Override
  public boolean wantsCargo() {
    // Permissive checks
    boolean flywheelOnTarget = onTarget();
    boolean turretOnTarget = m_turret.onTarget();
    boolean permissiveChecks = flywheelOnTarget && turretOnTarget;

    // Restrictive checks
    boolean turretTracking = m_turret.isTracking();
    boolean turretNotMoving = m_turret.notMoving();
    boolean driveNotSpinning = Math.abs(m_sensors.navx.getYawRate()) <= p_shooterSpinLimit.getValue();
    boolean driveNotAccelerating = m_drive.getAccelerationEstimate() <= p_shooterAccelerationLimit.getValue();
    boolean hoodNotMoving = !m_hood.isMoving();
    boolean highShotProbability = m_shotProbabilitySupplier.getAsDouble() >= p_shooterProbabilityLimit.getValue();
    boolean untimedChecks = permissiveChecks && turretTracking && turretNotMoving && driveNotSpinning && driveNotAccelerating && hoodNotMoving && highShotProbability;
    boolean restrictiveChecks = false;
    if (untimedChecks && m_restrictiveCheckTimer.hasElapsed(p_restrictiveCheckTime.getValue())) {
      restrictiveChecks = true;
    } else if (!untimedChecks) {
      m_restrictiveCheckTimer.reset();
    }
    

    boolean wantsCargo = (m_active == ActiveMode.ACTIVE_PERMISSIVE && permissiveChecks) || (m_active == ActiveMode.ACTIVE_RESTRICTIVE && restrictiveChecks);

    // if (m_active == ActiveMode.ACTIVE_PERMISSIVE && !wantsCargo) {
    //   String printString = "@Permissive Blocked: ";

    //   if (!flywheelOnTarget) {
    //     printString += "<Flywheel error is " + convertMotorTicksToRPM(m_flywheel.getClosedLoopError()) + "> ";
    //   }

    //   if (!turretOnTarget) {
    //     printString += "<Turret target is " + m_turret.getTarget() + " but facing is " + m_turret.getFacing() + "> ";
    //   }

    //   // System.out.println(printString);
    // } else if (m_active == ActiveMode.ACTIVE_RESTRICTIVE && !wantsCargo) {
    //   String printString = "@Restrictive Blocked: ";

    //   if (!flywheelOnTarget) {
    //     printString += "<Flywheel error is " + convertMotorTicksToRPM(m_flywheel.getClosedLoopError()) + "> ";
    //   }

    //   if (!turretTracking) {
    //     printString += "<Turret is not tracking> ";
    //   } else if (!turretOnTarget) {
    //     printString += "<Turret target is " + m_turret.getTarget() + " but facing is " + m_turret.getFacing() + "> ";
    //   }

    //   if (!driveNotSpinning) {
    //     printString += "<Yaw Rate is " + m_sensors.navx.getYawRate() + "> ";
    //   }

    //   if (!driveNotAccelerating) {
    //     printString += "<Acceleration estimate is " + m_drive.getAccelerationEstimate() + "> ";
    //   }

    //   if (!hoodNotMoving) {
    //     printString += "<Hood state is " + m_hood.getHoodState() + "> ";
    //   }

    //   if (!highShotProbability) {
    //     printString += "<Shot probability is " + m_shotProbabilitySupplier.getAsDouble() + "> ";
    //   }

    //   if (untimedChecks) {
    //     printString += "<Only " + m_restrictiveCheckTimer.get() + " has elapsed> ";
    //   }

    //   // System.out.println(printString);
    // } else 
    if (m_active != ActiveMode.DEACTIVATED && wantsCargo && !m_wantedCargo) {
      double flywheelVelocity = convertMotorTicksToRPM(m_flywheel.getSelectedSensorVelocity());
      m_ros_interface.signalShot(m_turret.getFacing(), m_rosDistance, flywheelVelocity);

      // System.out.println("@Shot: <Mode is " + m_active.toString() + "> " 
      //                  + "<Distance is " + m_limelightDistance + " from limelight and " + m_rosDistance + " from ROS, flywheel speed is " + flywheelVelocity + "> " 
      //                  + "<Angle is " + m_limelightAngle + " from limelight and " + m_rosAngle + " from ROS, turret angle is" + m_turret.getFacing() + "> " 
      //                  + "<Drive speed is " + m_drive.getStraightSpeed() + " with an acceleration estimate of " + m_drive.getAccelerationEstimate() + " and a yaw rate of " + m_sensors.navx.getYawRate() + "> " 
      //                  + "<Shot probability is " + m_ros_interface.getShooterProbability() + ">");
    }

    m_wantedCargo = wantsCargo;
    return wantsCargo;
  }

  public void registerLimelightTarget(double distance, double angle) {
    m_limelightDistance = distance;
    m_limelightAngle = angle;
  }
  
  public void registerROSTarget(double distance, double angle) {
    m_rosDistance = distance;
    m_rosAngle = angle;
  }

  @Override
  public void periodic() {

    if (!RobotContainer.isPublishingEnabled()) {
      return;
    }
    
    SmartDashboard.putNumber("Shooter Flywheel Velocity", convertMotorTicksToRPM(m_flywheel.getSelectedSensorVelocity())
        );
    SmartDashboard.putNumber("Shooter Flywheel Commanded Velocity",
        convertMotorTicksToRPM(m_flywheel.getClosedLoopTarget()));
    SmartDashboard.putBoolean("Shooter Flywheel On Target", onTarget());
    SmartDashboard.putNumber("Shooter Flywheel Error", m_flywheel.getClosedLoopError());
    // SmartDashboard.putNumber("Flywheel Speed from Limelight", calcSpeedFromDistance(??));
    SmartDashboard.putBoolean("Shooter Wants Cargo", wantsCargo());
  }
}
