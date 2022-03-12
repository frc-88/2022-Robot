// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
  private Hood m_hood;
  private Drive m_drive;
  private Sensors m_sensors;
  private Boolean m_active = false;
  private int m_cargoWaitCount = 0;
  private boolean m_sourcesHadCargoLastCheck = false;
  private long m_lastCargoEnteredShooter = 0;

  private static final double FLYWHEEL_RATIO = 1;

  private final ValueInterpolator hoodDownInterpolator = new ValueInterpolator(
      new ValueInterpolator.ValuePair(68.5, 1950),
      new ValueInterpolator.ValuePair(85, 2000),
      new ValueInterpolator.ValuePair(106.5, 2050));

  private final ValueInterpolator hoodUpInterpolator = new ValueInterpolator(
      new ValueInterpolator.ValuePair(116, 2300),
      new ValueInterpolator.ValuePair(131, 2400),
      new ValueInterpolator.ValuePair(150.5, 2550),
      new ValueInterpolator.ValuePair(171, 2850),
      new ValueInterpolator.ValuePair(203.5, 3250),
      new ValueInterpolator.ValuePair(241, 3600),
      new ValueInterpolator.ValuePair(278.5, 4350));

  // Preferences
  private PIDPreferenceConstants p_flywheelPID = new PIDPreferenceConstants("Shooter PID", 0.0, 0.0, 0.0, 0.047, 0.0,
      0.0, 0.0);
  private DoublePreferenceConstant p_flywheelIdle = new DoublePreferenceConstant("Shooter Idle Speed", 1300.0);
  private DoublePreferenceConstant p_flywheelFenderShot = new DoublePreferenceConstant("Shooter Fender Shot", 2100.0);
  private DoublePreferenceConstant p_flywheelBlindUp = new DoublePreferenceConstant("Shooter Blind Up Speed", 5000.0);
  private DoublePreferenceConstant p_flywheelBlindDown = new DoublePreferenceConstant("Shooter Blind Down Speed", 5000.0);
  private DoublePreferenceConstant p_shooterReady = new DoublePreferenceConstant("Shooter Pause (s)", 0.5);
  private DoublePreferenceConstant p_cargoInShooter = new DoublePreferenceConstant("Cargo In Shooter (s)", 0.2);

  /** Creates a new Shooter. */
  public Shooter(Hood hood, Drive drive, CargoSource[] sources, Sensors sensors) {
    m_hood = hood;
    m_drive = drive;
    m_sources = sources;
    m_sensors = sensors;

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

  public void setFlywheelSpeedAuto() {
    if (m_sensors.limelight.hasTarget()) {
      m_flywheel.set(TalonFXControlMode.Velocity, convertRPMsToMotorTicks(calcSpeedFromDistance()));
    } else if (!m_sensors.limelight.hasTarget() && sourcesHaveCargo()) {
      m_flywheel.set(TalonFXControlMode.Velocity, convertRPMsToMotorTicks(p_flywheelFenderShot.getValue()));
    } else if (!m_active) {
      m_flywheel.set(TalonFXControlMode.Velocity, convertRPMsToMotorTicks(p_flywheelIdle.getValue()));
    }
  }  

  private double calcSpeedFromDistance() {
    return m_sensors.limelight.hasTarget() 
      ? m_hood.isUp()
        ? hoodUpInterpolator.getInterpolatedValue(m_sensors.limelight.calcDistanceToTarget() + Constants.FIELD_UPPER_HUB_RADIUS)
        : hoodDownInterpolator.getInterpolatedValue(m_sensors.limelight.calcDistanceToTarget() + Constants.FIELD_UPPER_HUB_RADIUS)
      : m_hood.isUp() 
        ? p_flywheelBlindUp.getValue() 
        : p_flywheelBlindDown.getValue();
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

  private boolean sourcesHaveCargo() {
    boolean hasCargo = false;

    for (CargoSource source : m_sources) {
      hasCargo = hasCargo || source.hasCargo();
    }

    return hasCargo;
  }

  private boolean isFlywheelReady() {
    boolean ready = false;

    if (m_sources[0].hasCargo()) {
      ready = m_cargoWaitCount++ > p_shooterReady.getValue() * 50;
    } else {
      m_cargoWaitCount = 0;
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
    return m_active && isFlywheelReady() && onTarget() && !m_hood.isMoving() && m_sensors.limelight.onTarget();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Flywheel Velocity",
        convertMotorTicksToRPM(m_flywheel.getSelectedSensorVelocity()));
    SmartDashboard.putBoolean("Shooter Flywheel On Target", onTarget());
    SmartDashboard.putNumber("Flywheel Speed from Limelight", calcSpeedFromDistance());
    SmartDashboard.putBoolean("isFlywheelReady", isFlywheelReady());
    SmartDashboard.putBoolean("Shooter Wants Cargo", wantsCargo());

    if (m_active && !sourcesHaveCargo() && m_sourcesHadCargoLastCheck) {
      m_lastCargoEnteredShooter = RobotController.getFPGATime();
    }

    if (m_active && (RobotController.getFPGATime() - m_lastCargoEnteredShooter) <= p_cargoInShooter.getValue() * 1_000_000) {
      m_drive.lockDrive();
    } else {
      m_drive.unlockDrive();
    }
  }
}
