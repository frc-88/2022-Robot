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
import frc.robot.RobotContainer;
import frc.robot.util.NumberCache;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

/**
 * Shots from the hangar,
 * from against the fender and
 * everywhere between
 */
public class Hood extends SubsystemBase {
  private TalonFX m_hood = new TalonFX(Constants.HOOD_ID, "1");
  private Sensors m_sensors;

  private static final double HOOD_RATIO = 20;
  private static final double HOOD_LOWERED = 7.5;
  private static final double HOOD_RAISED = 27.0;
  private static final double HOOD_MID = 15.;
  private static final double HOOD_SETPOINT_TOLERANCE = 3;
  private static final double HOOD_CALIBRATION_COLLECT_SIZE = 20;
  private static final double HOOD_CALIBRATION_TOLERANCE = 0.5;
  private double m_hoodCalibrationStartValue = 0;
  private int m_hoodCalibrationCollectsDone = 0;

  private boolean isCoasting = false;

  private HoodState m_hoodState = HoodState.CALIBRATING;

  private static enum HoodState {
    CALIBRATING,
    RAISING,
    RAISED,
    MIDING,
    MIDED,
    LOWERING,
    LOWERED
  }

  private DoublePreferenceConstant p_continuousCurrentLimit = new DoublePreferenceConstant("Hood Continuous Current",
      10);
  private DoublePreferenceConstant p_triggerCurrentLimit = new DoublePreferenceConstant("Hood Trigger Current", 80);
  private DoublePreferenceConstant p_triggerDuration = new DoublePreferenceConstant("Hood Trigger Current Duration",
      0.002);
  private DoublePreferenceConstant p_hoodMaxVelocity = new DoublePreferenceConstant("Hood Max Velocity", 360);
  private DoublePreferenceConstant p_hoodMaxAcceleration = new DoublePreferenceConstant("Hood Max Acceleration", 1080);
  private PIDPreferenceConstants p_hoodPID = new PIDPreferenceConstants("Hood", 0, 0, 0, 0, 0, 0, 0);
  private DoublePreferenceConstant p_hoodArbitraryF = new DoublePreferenceConstant("Hood Arbitrary F", 0.0);
  private DoublePreferenceConstant p_hoodSpeed = new DoublePreferenceConstant("Hood Speed", 0.0);
  private DoublePreferenceConstant p_hoodMidDistance = new DoublePreferenceConstant("Hood Mid Distance", 80.);
  private DoublePreferenceConstant p_hoodUpDistance = new DoublePreferenceConstant("Hood Up Distance", 110.);

  public Hood(Sensors sensors) {
    m_sensors = sensors;

    configureHood();

    p_continuousCurrentLimit.addChangeHandler((Double unused) -> configureHood());
    p_triggerCurrentLimit.addChangeHandler((Double unused) -> configureHood());
    p_triggerDuration.addChangeHandler((Double unused) -> configureHood());
    p_hoodMaxVelocity.addChangeHandler((Double unused) -> configureHood());
    p_hoodMaxAcceleration.addChangeHandler((Double unused) -> configureHood());
    p_hoodPID.addChangeHandler((Double unused) -> configureHood());
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
    config.slot0.integralZone = p_hoodPID.getIZone().getValue();
    config.slot0.maxIntegralAccumulator = p_hoodPID.getIMax().getValue();
    config.statorCurrLimit = new StatorCurrentLimitConfiguration(true, p_continuousCurrentLimit.getValue(),
        p_triggerCurrentLimit.getValue(), p_triggerDuration.getValue());
    m_hood.configAllSettings(config);
    m_hood.setInverted(InvertType.InvertMotorOutput);
  }

  public void hoodAuto(double target_dist) {
    double upDistance = p_hoodUpDistance.getValue();
    if (target_dist > 0.0) {
      if (!(m_hoodState == HoodState.LOWERING || m_hoodState == HoodState.LOWERED) && target_dist < upDistance - 1.5) {
        lowerHood();
      } else if (!(m_hoodState == HoodState.RAISING || m_hoodState == HoodState.RAISED) && target_dist > upDistance + 1.5) {
        raiseHood();
      } else if (m_hoodState == HoodState.RAISING || m_hoodState == HoodState.RAISED) {
        raiseHood();
      } else {
        lowerHood();
      }
    } else {
      lowerHood();
    }
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
      case MIDING:
      case MIDED:
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
      case MIDING:
      case MIDED:
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

  public void midHood() {
    if (m_hoodState == HoodState.CALIBRATING) {
      lowerHood();
      return;
    }

    setHoodMotionMagic(HOOD_MID);
    if (Math.abs(HOOD_MID - getHoodPosition()) <= HOOD_SETPOINT_TOLERANCE) {
      m_hoodState = HoodState.MIDED;
    } else {
      m_hoodState = HoodState.MIDING;
    }
  }

  public String getHoodState() {
    return m_hoodState.toString();
  }

  public boolean isUp() {
    return m_hoodState == HoodState.RAISED;
  }

  public boolean isDown() {
    return m_hoodState == HoodState.LOWERED;
  }

  public boolean isMid() {
    return m_hoodState == HoodState.MIDED;
  }

  public boolean isMoving() {
    return (m_hoodState != HoodState.RAISED && m_hoodState != HoodState.LOWERED && m_hoodState != HoodState.MIDED && m_hoodState != HoodState.CALIBRATING);
  }

  public void setHoodPercentOut(int direction) {
    m_hood.set(TalonFXControlMode.PercentOutput, p_hoodSpeed.getValue() * direction);
  }

  private void setHoodMotionMagic(double setpoint) {
    m_hood.set(TalonFXControlMode.MotionMagic, convertHoodPositionToMotor(setpoint), DemandType.ArbitraryFeedForward,
        p_hoodArbitraryF.getValue());
  }

  public double getHoodPosition() {
    if (NumberCache.hasValue("Hood Position")) {
      return NumberCache.getValue("Hood Position");
    }

    return NumberCache.pushValue("Hood Position", convertMotorPositionToHood(m_hood.getSelectedSensorPosition()));
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

  @Override
  public void periodic() {
    if (m_sensors.isCoastButtonPressed()) {
      if (!isCoasting) {
        m_hood.setNeutralMode(NeutralMode.Coast);
        isCoasting = true;
      }
    } else {
      if (isCoasting) {
        m_hood.setNeutralMode(NeutralMode.Brake);
        isCoasting = false;
      }
    }

    if (!RobotContainer.isPublishingEnabled()) {
      return;
    }

    SmartDashboard.putNumber("Hood Position", getHoodPosition());
    SmartDashboard.putString("Hood State", m_hoodState.toString());
  }
}
