// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CargoTarget;
import frc.robot.util.ValueInterpolator;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;
import frc.robot.util.sensors.Limelight;

public class Shooter extends SubsystemBase implements CargoTarget {
  private TalonFX m_flywheel = new TalonFX(Constants.SHOOTER_FLYWHEEL_ID, "1");
  private TalonFX m_hood = new TalonFX(Constants.SHOOTER_HOOD_ID, "1");
  private Limelight m_limelight;
  private Boolean m_active = false;

  private static final double FLYWHEEL_RATIO = 3;
  private static final double HOOD_RATIO = 2;

  public static final double HOOD_LOWERED = 12.5;
  public static final double HOOD_RAISED = 37.5;

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
    new ValueInterpolator.ValuePair(100, 1000),
    new ValueInterpolator.ValuePair(200, 2000)
  );

  private final ValueInterpolator hoodUpInterpolator = new ValueInterpolator(
    new ValueInterpolator.ValuePair(100, 1000),
    new ValueInterpolator.ValuePair(200, 2000)
  );

  // Preferences
  private DoublePreferenceConstant p_continuousCurrentLimit = new DoublePreferenceConstant("Hood Continuous Current", 10);
  private DoublePreferenceConstant p_triggerCurrentLimit = new DoublePreferenceConstant("Hood Trigger Current", 80);
  private DoublePreferenceConstant p_triggerDuration = new DoublePreferenceConstant("Hood Trigger Current Duration", 0.002);
  private DoublePreferenceConstant p_hoodMaxVelocity = new DoublePreferenceConstant("Hood Max Velocity", 360);
  private DoublePreferenceConstant p_hoodMaxAcceleration = new DoublePreferenceConstant("Hood Max Acceleration", 1080);
  private PIDPreferenceConstants p_hoodPID = new PIDPreferenceConstants("Hood", 0, 0, 0, 0, 0, 0, 0);
  private DoublePreferenceConstant p_hoodSpeed = new DoublePreferenceConstant("Hood Speed", 0.0);
  private PIDPreferenceConstants p_flywheelPID = new PIDPreferenceConstants("Shooter PID", 0.0, 0.0, 0.0, 0.047, 0.0, 0.0, 0.0);

  /** Creates a new Shooter. */
  public Shooter(Limelight limelight) {
    m_limelight = limelight;
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

  public void setFlywheelSpeedFromLimelight() {
    if (m_limelight.hasTarget()) {
      m_flywheel.set(TalonFXControlMode.Velocity, convertRPMsToMotorTicks(getFlywheelSpeedFromLimelight()));
    }
  }

  public boolean onTarget() {
    return Math.abs(convertMotorTicksToRPM(m_flywheel.getClosedLoopError())) < p_flywheelPID.getTolerance().getValue();
  }

  public void raiseHood() {
    m_limelight.setHood(true);
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

        m_hoodState = HoodState.RAISED;

        break;
    }
  }

  public void lowerHood() {
    m_limelight.setHood(false);
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

        m_hoodState = HoodState.RAISED;

        break;
    }
  }

  private void setHoodPercentOut(int direction) {
    m_hood.set(TalonFXControlMode.PercentOutput, p_hoodSpeed.getValue() * direction);
  }

  private void setHoodMotionMagic(double setpoint) {
    m_hood.set(TalonFXControlMode.MotionMagic, convertHoodPositionToMotor(setpoint));
  }

  public void activate() {
    m_active = true;
  }

  public void deactivate() {
    m_active = false;
  }

  @Override
  public boolean wantsCargo() {
    // return m_active && m_limelight.onTarget();
    return m_active;
  }

  public double getHoodPosition() {
    return convertHoodPositionToMotor(m_hood.getSelectedSensorPosition());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Flywheel Velocity", convertMotorTicksToRPM(m_flywheel.getSelectedSensorVelocity()));
    SmartDashboard.putBoolean("Shooter Flywheel On Target", onTarget());
    SmartDashboard.putNumber("Flywheel Speed from Limelight", getFlywheelSpeedFromLimelight());

    SmartDashboard.putNumber("Hood Position", getHoodPosition());
    SmartDashboard.putString("Hood State", m_hoodState.toString());
  }

  private double getFlywheelSpeedFromLimelight() {
    return m_limelight.isHoodUp() 
        ? hoodUpInterpolator.getInterpolatedValue(m_limelight.calcDistanceToTarget()) 
        : hoodDownInterpolator.getInterpolatedValue(m_limelight.calcDistanceToTarget());
  }

  private double convertMotorTicksToRPM(double motorVelocity) {
    return motorVelocity * 600 / (FLYWHEEL_RATIO * 2048);
  }

  private double convertRPMsToMotorTicks(double flywheelVelocity) {
    return flywheelVelocity * FLYWHEEL_RATIO * 2048 / 600;
  }
  

  private double convertMotorPositionToHood(double motorPosition) {
    return motorPosition / (FLYWHEEL_RATIO * 2048);
  }

  private double convertMotorVelocityToHood(double motorVelocity) {
    return convertMotorPositionToHood(motorVelocity) * 10;
  }

  private double convertHoodPositionToMotor(double hoodPosition) {
    return hoodPosition * HOOD_RATIO * 2048;
  }

  private double convertHoodVelocityToMotor(double hoodVelocity) {
    return convertHoodPositionToMotor(hoodVelocity) / 10;
  }
}
