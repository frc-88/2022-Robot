// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CargoSource;
import frc.robot.util.CargoTarget;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.IntPreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

/*
 * feed me some cargo
 * then I can pass it along
 * I can cargolize!
 */

public class Feeder extends SubsystemBase implements CargoSource, CargoTarget {
  private String m_feederName;
  private TalonFX m_feederMotor;
  private Servo m_blockerServo;
  private boolean m_hasBlocker;
  private DoublePreferenceConstant p_feederMaxVelocityFinding;
  private DoublePreferenceConstant p_feederMaxVelocityForcing;
  private DoublePreferenceConstant p_feederMaxAcceleration;
  private PIDPreferenceConstants p_feederPID;
  private PIDPreferenceConstants p_feederPIDVelocity;
  private DoublePreferenceConstant p_feederTargetPosition;
  private DoublePreferenceConstant p_blockerDown;
  private DoublePreferenceConstant p_blockerUp;
  private IntPreferenceConstant p_loseCargoCount;

  private boolean m_foundCargo = false;
  private int m_loseCargoCounter = 0;
  
  public Feeder(String feederName, int feederMotorId, boolean invert, int blockerId) {
    setName(feederName);
    m_feederName = feederName;
    m_feederMotor = new TalonFX(feederMotorId, "1");
    p_feederMaxVelocityFinding = new DoublePreferenceConstant(feederName + " Max Velocity Finding", 1700);
    p_feederMaxVelocityForcing = new DoublePreferenceConstant(feederName + " Max Velocity Forcing", 1700);
    p_feederMaxAcceleration = new DoublePreferenceConstant(feederName + " Max Acceleration", 50000);
    p_feederPID = new PIDPreferenceConstants(feederName, 0, 0, 0, 0, 0, 0, 0);
    p_feederPIDVelocity = new PIDPreferenceConstants(feederName + " Velocity", 0, 0, 0, 0, 0, 0, 0);
    p_feederTargetPosition = new DoublePreferenceConstant(feederName + " Target Position", 100);
    p_loseCargoCount = new IntPreferenceConstant(feederName + " Lose Cargo Count", 15);

    m_feederMotor.configFactoryDefault();
    m_feederMotor.setInverted(invert ? InvertType.InvertMotorOutput : InvertType.None);
    m_feederMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_feederMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    m_feederMotor.overrideLimitSwitchesEnable(false);
    m_feederMotor.setSelectedSensorPosition(0);
    m_feederMotor.configClearPositionOnLimitR(true, 0);
    configPID();

    p_feederMaxAcceleration.addChangeHandler((Double unused) -> configPID());
    p_feederPID.addChangeHandler((Double unused) -> configPID());
    p_feederPIDVelocity.addChangeHandler((Double unused) -> configPID());

    if (blockerId >= 0) {
      m_blockerServo = new Servo(blockerId);
      m_hasBlocker = true;
      p_blockerDown = new DoublePreferenceConstant("Blocker down", 0);
      p_blockerUp = new DoublePreferenceConstant("Blocker up", 90);
    } else {
      m_hasBlocker = false;
    }

  }

  public Feeder(String feederName, int feederMotorId, boolean invert) {
    this(feederName, feederMotorId, invert, -1);
  }

  private void configPID() {
    m_feederMotor.configMotionCruiseVelocity(p_feederMaxVelocityFinding.getValue());
    m_feederMotor.configMotionAcceleration(p_feederMaxAcceleration.getValue());
    m_feederMotor.config_kP(0, p_feederPID.getKP().getValue());
    m_feederMotor.config_kI(0, p_feederPID.getKI().getValue());
    m_feederMotor.config_kD(0, p_feederPID.getKD().getValue());
    m_feederMotor.config_kF(0, p_feederPID.getKF().getValue());
    m_feederMotor.config_IntegralZone(0, p_feederPID.getIZone().getValue());
    m_feederMotor.configMaxIntegralAccumulator(0, p_feederPID.getIMax().getValue());

    m_feederMotor.config_kP(1, p_feederPIDVelocity.getKP().getValue());
    m_feederMotor.config_kI(1, p_feederPIDVelocity.getKI().getValue());
    m_feederMotor.config_kD(1, p_feederPIDVelocity.getKD().getValue());
    m_feederMotor.config_kF(1, p_feederPIDVelocity.getKF().getValue());
    m_feederMotor.config_IntegralZone(1, p_feederPIDVelocity.getIZone().getValue());
    m_feederMotor.configMaxIntegralAccumulator(1, p_feederPIDVelocity.getIMax().getValue());
  }

  public void runUntilBallFound() {
    blockCargo();

    if (sensorTriggered()) {
      m_foundCargo = true;
      m_loseCargoCounter = 0;
      m_feederMotor.configClearPositionOnLimitR(false, 0);
    } else {
      m_feederMotor.configClearPositionOnLimitR(true, 0);
      if (m_feederMotor.getSelectedSensorPosition() > -5_000) {
        m_feederMotor.setSelectedSensorPosition(-10_000_000);
      }
    }

    if (m_foundCargo && !sensorTriggered() && m_loseCargoCounter++ > p_loseCargoCount.getValue()) {
      m_foundCargo = false;
    }

    m_feederMotor.selectProfileSlot(0, 0);
    m_feederMotor.set(TalonFXControlMode.MotionMagic, p_feederTargetPosition.getValue());
  }

  public void forceForwards() {
    unblockCargo();

    if (m_foundCargo && !sensorTriggered()) {
      m_foundCargo = false;
    }

    m_feederMotor.configClearPositionOnLimitR(false, 0);

    m_feederMotor.selectProfileSlot(1, 0);
    m_feederMotor.set(TalonFXControlMode.Velocity, p_feederMaxVelocityForcing.getValue());
  }

  public void forceReverse() {
    unblockCargo();

    if (m_foundCargo && !sensorTriggered()) {
      m_foundCargo = false;
    }

    m_feederMotor.configClearPositionOnLimitR(false, 0);

    m_feederMotor.selectProfileSlot(1, 0);
    m_feederMotor.set(TalonFXControlMode.Velocity, -p_feederMaxVelocityForcing.getValue());
  }

  public void stop() {
    blockCargo();

    if (m_foundCargo && !sensorTriggered() && m_loseCargoCounter++ > p_loseCargoCount.getValue()) {
      m_foundCargo = false;
    }

    m_feederMotor.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public boolean hasCargo() {
    return m_foundCargo || sensorTriggered();
  }

  private boolean sensorTriggered() {
    return m_feederMotor.isRevLimitSwitchClosed() == 1;
  }

  @Override
  public boolean wantsCargo() {
    return !hasCargo();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean(m_feederName + ":hasCargo?", hasCargo());
  }

  private void unblockCargo() {
    if (m_hasBlocker) {
      m_blockerServo.setAngle(p_blockerDown.getValue());
    }
  }

  private void blockCargo() {
    if (m_hasBlocker) {
      m_blockerServo.setAngle(p_blockerUp.getValue());
    }
  }
}
