// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  private Boolean m_active = false;
  private Timer m_cargoWaitTimer = new Timer();
  private boolean m_cargoWaiting = false;
  private boolean m_sourcesHadCargoLastCheck = false;
  private long m_lastCargoEnteredShooter = 0;

  private static final double FLYWHEEL_RATIO = 1;

  private final ValueInterpolator hoodDownInterpolator = new ValueInterpolator(
      new ValueInterpolator.ValuePair(62.2, 2000),
      new ValueInterpolator.ValuePair(70.9, 2050),
      new ValueInterpolator.ValuePair(82.3, 2085),
      new ValueInterpolator.ValuePair(94.9, 2150),
      new ValueInterpolator.ValuePair(109.1, 2200),
      new ValueInterpolator.ValuePair(119.7, 2500));

  private final ValueInterpolator hoodMidInterpolator = new ValueInterpolator(
      new ValueInterpolator.ValuePair(85.5, 2200),
      new ValueInterpolator.ValuePair(102, 2200),
      new ValueInterpolator.ValuePair(117, 2250),
      new ValueInterpolator.ValuePair(134, 2375),
      new ValueInterpolator.ValuePair(152, 2800),
      new ValueInterpolator.ValuePair(166, 3100));

  private final ValueInterpolator hoodUpInterpolator = new ValueInterpolator(
      new ValueInterpolator.ValuePair(108.3, 2365),
      new ValueInterpolator.ValuePair(120.1, 2450),
      new ValueInterpolator.ValuePair(129.5, 2450),
      new ValueInterpolator.ValuePair(141.7, 2475),
      new ValueInterpolator.ValuePair(168.9, 2600),
      new ValueInterpolator.ValuePair(179.1, 2700),
      new ValueInterpolator.ValuePair(191.7, 2875),
      new ValueInterpolator.ValuePair(226.0, 3325),
      new ValueInterpolator.ValuePair(282.3, 4275));

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

  /** Creates a new Shooter. */
  public Shooter(Sensors sensors, Hood hood, Drive drive, Turret turret, CargoSource[] sources) {
    m_sensors = sensors;
    m_hood = hood;
    m_drive = drive;
    m_turret = turret;
    m_sources = sources;

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

  public void activate() {
    m_active = true;
  }

  public void deactivate() {
    m_active = false;
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

  private boolean isFlywheelReady() {
    boolean ready = false;

    if (m_sources[0].hasCargo()) {
      if (!m_cargoWaiting) {
        m_cargoWaiting = true;
        m_cargoWaitTimer.reset();
        m_cargoWaitTimer.start();
      } else if (m_cargoWaitTimer.get() > p_shooterReady.getValue()) {
        ready = true;
      }
    } else {
      ready = true;
      m_cargoWaiting = false;
    }

    return ready;
  }

  @Override
  public boolean wantsCargo() {
    // TODO combine some or all of these conditions and return that
    // m_active - shooter button pushed
    // onTarget()
    // m_limelight.onTarget()
    // m_hoodState == HoodState.LOWERED || m_hoodState == HoodState.RAISED
    // boolean wantsCargo = (m_active && isFlywheelReady() && onTarget() && !m_hood.isMoving());
    boolean isFlywheelReady = isFlywheelReady();
    boolean onTarget = onTarget();
    boolean turretOnTarget = m_turret.onTarget();
    boolean driveSpinning = Math.abs(m_sensors.navx.getYawRate()) < 90.;
    boolean wantsCargo = (m_active && isFlywheelReady && onTarget && turretOnTarget);

    if (m_active && !wantsCargo) {
      System.out.println("***Shot blocked***");
      System.out.println("isFlywheelReasdy:" + isFlywheelReady);
      System.out.println("onTarget:" + onTarget);
      System.out.println("Turret onTarget" + turretOnTarget);
    }

    if (m_active && wantsCargo) {
      System.out.println("!!!SHOOT!!!");
    }

    return wantsCargo;
  }

  @Override
  public void periodic() {
    if (m_active && !sourcesHaveCargo() && m_sourcesHadCargoLastCheck) {
      m_lastCargoEnteredShooter = RobotController.getFPGATime();
    }

    if (RobotController.getFPGATime() - m_lastCargoEnteredShooter < p_cargoInShooter.getValue() * 1_000_000) {
      m_drive.lockDrive();
    } else {
      m_drive.unlockDrive();
    }

    if (!RobotContainer.isPublishingEnabled()) {
      return;
    }
    
    SmartDashboard.putNumber("Shooter Flywheel Velocity",
        convertMotorTicksToRPM(m_flywheel.getSelectedSensorVelocity()));
    SmartDashboard.putNumber("Shooter Flywheel Commanded Velocity",
        convertMotorTicksToRPM(m_flywheel.getClosedLoopTarget()));
    SmartDashboard.putBoolean("Shooter Flywheel On Target", onTarget());
    // SmartDashboard.putNumber("Flywheel Speed from Limelight", calcSpeedFromDistance(??));
    SmartDashboard.putBoolean("isFlywheelReady", isFlywheelReady());
    SmartDashboard.putBoolean("Shooter Wants Cargo", wantsCargo());
  }
}
