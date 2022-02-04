// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Turret extends SubsystemBase {
  private TalonFX m_turret;
  private CANCoder m_encoder;
  private DoublePreferenceConstant m_turretZeroPositionPref = new DoublePreferenceConstant("Turret Zero",  Constants.TURRET_DEFAULT_ZERO);
  private DoublePreferenceConstant m_turretForwardLimitPref = new DoublePreferenceConstant("Turret Forward Limit", Constants.TURRET_DEFAULT_FWD_LIMIT);
  private DoublePreferenceConstant m_turretReverseLimitPref = new DoublePreferenceConstant("Turret Reverse Limit", Constants.TURRET_DEFAULT_REV_LIMIT);
  private boolean m_tracking = false;

  /** Creates a new Turret. */
  public Turret() {
    m_turret = new TalonFX(Constants.TURRET_MOTOR_ID);
    m_encoder = new CANCoder(Constants.TURRET_ENCODER_ID);

    // TODO - better config, cancoder config?
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    config.forwardSoftLimitThreshold = m_turretForwardLimitPref.getValue();
    config.forwardSoftLimitEnable = true;
    config.reverseSoftLimitThreshold = m_turretReverseLimitPref.getValue();
    config.reverseSoftLimitEnable = true;
    config.peakOutputForward = 1.0;
    config.peakOutputReverse = -1.0;
    config.nominalOutputForward = 0;
    config.nominalOutputReverse = 0;
    config.neutralDeadband = 0.001;
    // config.slot0.kP = 0.00000;
    // config.slot0.kI = 0.00000;
    // config.slot0.kD = 0.00000;
    // config.slot0.kF = 1.00000;
    m_turret.configAllSettings(config);

    CANCoderConfiguration encConfig = new CANCoderConfiguration();
    m_encoder.configAllSettings(encConfig);

    // initialize internal sensor to correct absolute position when we wake up
    m_turret.setSelectedSensorPosition(encoderPostionToTurretFacing(m_encoder.getAbsolutePosition()));
  }

  public void rawMotor(double percentOutput) {
    m_turret.set(TalonFXControlMode.PercentOutput, percentOutput);
  }

  public void goToPosition(double position) {
    m_turret.set(TalonFXControlMode.MotionMagic, position);
  }

  public double getPosition() {
    return m_turret.getSelectedSensorPosition();
  }

  public boolean isSynchronized() {
    return Math.abs(getPosition() - encoderPostionToTurretFacing(m_encoder.getAbsolutePosition())) < 
       Constants.TURRET_SYNCRONIZATION_THRESHOLD;
  }

  public void startTracking() {
    m_tracking = true;
  }

  public void stopTracking() {
    m_tracking = false;
  }

  public boolean isTracking() {
    return m_tracking;
  }

  private double encoderPostionToTurretFacing (double encPosition) {
    return (encPosition - m_turretZeroPositionPref.getValue()) * Constants.TURRET_CANCODER_CONV;
  }

  private double turretFacingToEncoderPostion (double turretFacing) {
    return turretFacing / Constants.TURRET_CANCODER_CONV + m_turretZeroPositionPref.getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret:CANCoder", m_encoder.getAbsolutePosition());
    SmartDashboard.putNumber("Turret:Position", getPosition());
  }
}

