// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

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
  private TalonFX m_hood = new TalonFX(Constants.SHOOTER_HOOD_ID, "1");
  private CargoSource[] m_sources;
  private Sensors m_sensors;
  private Boolean m_active = false;

  private static final double FLYWHEEL_RATIO = 1;
  private static final double HOOD_RATIO = 20;

  public static final double HOOD_LOWERED = 0.0;
  public static final double HOOD_RAISED = 27.0;

  private static final double HOOD_SETPOINT_TOLERANCE = 3;

  private static final double HOOD_CALIBRATION_COLLECT_SIZE = 25;
  private static final double HOOD_CALIBRATION_TOLERANCE = 0.5;
  private double m_hoodCalibrationStartValue = 0;
  private int m_hoodCalibrationCollectsDone = 0;

  private HoodState m_hoodState = HoodState.CALIBRATING;
  private static enum HoodState {
    CALIBRATING,
    RAISING,
    RAISED,
    LOWERING,
    LOWERED
  }

  private final ValueInterpolator hoodDownInterpolator = new ValueInterpolator(
      new ValueInterpolator.ValuePair(28, 2000),
      new ValueInterpolator.ValuePair(92, 2000)
      );
    
      private final ValueInterpolator hoodUpInterpolator = new ValueInterpolator(
        new ValueInterpolator.ValuePair(95, 2200),
        new ValueInterpolator.ValuePair(100, 2300),
        new ValueInterpolator.ValuePair(105, 2400),
        new ValueInterpolator.ValuePair(136, 3000)
      );
    
  // Preferences
  private DoublePreferenceConstant p_continuousCurrentLimit = new DoublePreferenceConstant("Hood Continuous Current", 10);
  private DoublePreferenceConstant p_triggerCurrentLimit = new DoublePreferenceConstant("Hood Trigger Current", 80);
  private DoublePreferenceConstant p_triggerDuration = new DoublePreferenceConstant("Hood Trigger Current Duration", 0.002);
  private DoublePreferenceConstant p_hoodMaxVelocity = new DoublePreferenceConstant("Hood Max Velocity", 360);
  private DoublePreferenceConstant p_hoodMaxAcceleration = new DoublePreferenceConstant("Hood Max Acceleration", 1080);
  private PIDPreferenceConstants p_hoodPID = new PIDPreferenceConstants("Hood", 0, 0, 0, 0, 0, 0, 0);
  private DoublePreferenceConstant p_hoodArbitraryF = new DoublePreferenceConstant("Hood Arbitrary F", 0.0);
  private DoublePreferenceConstant p_hoodSpeed = new DoublePreferenceConstant("Hood Speed", 0.0);
  private PIDPreferenceConstants p_flywheelPID = new PIDPreferenceConstants("Shooter PID", 0.0, 0.0, 0.0, 0.047, 0.0, 0.0, 0.0);
  private DoublePreferenceConstant p_flywheelIdle = new DoublePreferenceConstant("Shooter Idle Speed", 5000.0);

  /** Creates a new Shooter. */
  public Shooter(CargoSource [] sources, Sensors sensors) {
    m_sources = sources;
    m_sensors = sensors;

    configureFlywheel();
    configureHood();

    p_flywheelPID.addChangeHandler((Double unused) -> configureFlywheel());
    p_continuousCurrentLimit.addChangeHandler((Double unused) -> configureHood());
    p_triggerCurrentLimit.addChangeHandler((Double unused) -> configureHood());
    p_triggerDuration.addChangeHandler((Double unused) -> configureHood());
    p_hoodMaxVelocity.addChangeHandler((Double unused) -> configureHood());
    p_hoodMaxAcceleration.addChangeHandler((Double unused) -> configureHood());
    p_hoodPID.addChangeHandler((Double unused) -> configureHood());
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

  private void configureHood() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.neutralDeadband = 0;
    config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    config.motionCruiseVelocity = convertHoodVelocityToMotor(p_hoodMaxVelocity.getValue());
    config.motionAcceleration = convertHoodVelocityToMotor(p_hoodMaxAcceleration.getValue());
    config.slot0.kP = p_hoodPID.getKP().getValue();
    config.slot0.kI = p_hoodPID.getKI().getValue();
    config.slot0.kD = p_hoodPID.getKD().getValue();
    config.slot0.kF = p_hoodPID.getKF().getValue();
    config.slot0.integralZone= p_hoodPID.getIZone().getValue();
    config.slot0.maxIntegralAccumulator = p_hoodPID.getIMax().getValue();
    config.statorCurrLimit = new StatorCurrentLimitConfiguration(true, p_continuousCurrentLimit.getValue(),
        p_triggerCurrentLimit.getValue(), p_triggerDuration.getValue());
    m_hood.configAllSettings(config);
    m_hood.setInverted(InvertType.InvertMotorOutput);
  }

  public void setFlywheelSpeed(double speed) {
    m_flywheel.set(TalonFXControlMode.Velocity, convertRPMsToMotorTicks(speed));
  }

  public void setFlywheelRaw(double percentOutput) {
    m_flywheel.set(TalonFXControlMode.PercentOutput, percentOutput);
  }

  public void setFlywheelSpeedAuto() {
    m_flywheel.set(TalonFXControlMode.Velocity, convertRPMsToMotorTicks(getFlywheelSpeedFromLimelight()));
  }

  public boolean onTarget() {
    return Math.abs(convertMotorTicksToRPM(m_flywheel.getClosedLoopError())) < p_flywheelPID.getTolerance().getValue();
  }

  public void raiseHood() {
    switch (m_hoodState) {
      case CALIBRATING:
        setHoodPercentOut(1);

        double currentPosition = getHoodPosition();
        if (Math.abs(currentPosition - m_hoodCalibrationStartValue) <= HOOD_CALIBRATION_TOLERANCE) {
          m_hoodCalibrationCollectsDone++;
        } else {
          m_hoodCalibrationCollectsDone = 0;
          m_hoodCalibrationStartValue = currentPosition;
        }

        if (m_hoodCalibrationCollectsDone >= HOOD_CALIBRATION_COLLECT_SIZE) {
          m_hood.setSelectedSensorPosition(convertHoodPositionToMotor(HOOD_RAISED));
          m_hoodState = HoodState.RAISED;
        } else {
          m_hoodState = HoodState.CALIBRATING;
        }

        break;

      case LOWERING:
      case LOWERED:
      case RAISING:
        setHoodMotionMagic(HOOD_RAISED);

        if (Math.abs(HOOD_RAISED - getHoodPosition()) <= HOOD_SETPOINT_TOLERANCE) {
          m_hoodState = HoodState.RAISED;
        } else {
          m_hoodState = HoodState.RAISING;
        }

        break;

      case RAISED:
        setHoodPercentOut(1);
        break;
    }
  }

  public void lowerHood() {
    switch (m_hoodState) {
      case CALIBRATING:
        setHoodPercentOut(-1);

        double currentPosition = getHoodPosition();
        if (Math.abs(currentPosition - m_hoodCalibrationStartValue) <= HOOD_CALIBRATION_TOLERANCE) {
          m_hoodCalibrationCollectsDone++;
        } else {
          m_hoodCalibrationCollectsDone = 0;
          m_hoodCalibrationStartValue = currentPosition;
        }

        if (m_hoodCalibrationCollectsDone >= HOOD_CALIBRATION_COLLECT_SIZE) {
          m_hood.setSelectedSensorPosition(HOOD_LOWERED);
          m_hoodState = HoodState.LOWERED;
        } else {
          m_hoodState = HoodState.CALIBRATING;
        }

        break;

      case LOWERING:
      case RAISING:
      case RAISED:
        setHoodMotionMagic(HOOD_LOWERED);

        if (Math.abs(HOOD_LOWERED - getHoodPosition()) <= HOOD_SETPOINT_TOLERANCE) {
          m_hoodState = HoodState.LOWERED;
        } else {
          m_hoodState = HoodState.LOWERING;
        }

        break;

        case LOWERED:
        setHoodPercentOut(-1);
        break;
    }
  }

  public void setHoodPercentOut(int direction) {
    m_hood.set(TalonFXControlMode.PercentOutput, p_hoodSpeed.getValue() * direction);
  }

  private void setHoodMotionMagic(double setpoint) {
    m_hood.set(TalonFXControlMode.MotionMagic, convertHoodPositionToMotor(setpoint), DemandType.ArbitraryFeedForward, p_hoodArbitraryF.getValue());
  }

  public void activate() {
    m_active = true;
  }

  public void deactivate() {
    m_active = false;
  }

  private boolean sourcesHaveCargo() {
    boolean hasCargo = false;

    for (CargoSource source : m_sources) {
      hasCargo = hasCargo || source.hasCargo();
    }

    return hasCargo;
  }

  @Override
  public boolean wantsCargo() {
    // TODO combine some or all of these conditions and return that
    // m_active - shooter button pushed
    // onTarget()
    // m_limelight.onTarget()
    // m_hoodState == HoodState.LOWERED || m_hoodState == HoodState.RAISED
    return m_active;
  }

  public double getHoodPosition() {
    return convertMotorPositionToHood(m_hood.getSelectedSensorPosition());
  }

  @Override
  public void periodic() {
    if (m_sensors.isCoastButtonPressed()) {
        m_hood.setNeutralMode(NeutralMode.Coast);
    } else {
        m_hood.setNeutralMode(NeutralMode.Brake);
    }

    SmartDashboard.putNumber("Shooter Flywheel Velocity", convertMotorTicksToRPM(m_flywheel.getSelectedSensorVelocity()));
    SmartDashboard.putBoolean("Shooter Flywheel On Target", onTarget());
    SmartDashboard.putNumber("Flywheel Speed from Limelight", getFlywheelSpeedFromLimelight());

    SmartDashboard.putNumber("Hood Position", getHoodPosition());
    SmartDashboard.putString("Hood State", m_hoodState.toString());
  }

  private double getFlywheelSpeedFromLimelight() {
    if (!m_sensors.limelight.hasTarget()) {
      return (m_hoodState == HoodState.RAISED) ? 2300 : 2000;
    }
    return (m_hoodState == HoodState.RAISED)
        ? hoodUpInterpolator.getInterpolatedValue(m_sensors.limelight.calcDistanceToTarget())
        : hoodDownInterpolator.getInterpolatedValue(m_sensors.limelight.calcDistanceToTarget());
  }

  private double convertMotorTicksToRPM(double motorVelocity) {
    return motorVelocity * 600 / (FLYWHEEL_RATIO * 2048);
  }

  private double convertRPMsToMotorTicks(double flywheelVelocity) {
    return flywheelVelocity * FLYWHEEL_RATIO * 2048 / 600;
  }


  private double convertMotorPositionToHood(double motorPosition) {
    return motorPosition / (HOOD_RATIO * 2048.) * 360.0;
  }

  private double convertMotorVelocityToHood(double motorVelocity) {
    return convertMotorPositionToHood(motorVelocity) * 10;
  }

  private double convertHoodPositionToMotor(double hoodPosition) {
    return hoodPosition / 360.0 * HOOD_RATIO * 2048.;
  }

  private double convertHoodVelocityToMotor(double hoodVelocity) {
    return convertHoodPositionToMotor(hoodVelocity) / 10;
  }
}
