// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Turret extends SubsystemBase {
  private TalonFX m_turret = new TalonFX(Constants.TURRET_MOTOR_ID, "1");
  private CANCoder m_cancoder = new CANCoder(Constants.TURRET_CANCODER_ID, "1");
  // Preferences
  private DoublePreferenceConstant p_zeroPosition = new DoublePreferenceConstant("Turret Zero", Constants.TURRET_ZERO_DFT);
  private DoublePreferenceConstant p_nominalForward = new DoublePreferenceConstant("Turret Nominal Forward", Constants.TURRET_NOMINAL_FWD_DFT);
  private DoublePreferenceConstant p_nominalReverse = new DoublePreferenceConstant("Turret Nominal Reverse", Constants.TURRET_NOMINAL_REV_DFT);
  private DoublePreferenceConstant p_forwardLimit = new DoublePreferenceConstant("Turret Forward Limit", Constants.TURRET_FWD_LIMIT_DFT);
  private DoublePreferenceConstant p_reverseLimit = new DoublePreferenceConstant("Turret Reverse Limit", Constants.TURRET_REV_LIMIT_DFT);
  private DoublePreferenceConstant p_limitBuffer = new DoublePreferenceConstant("Turret Limit Buffer", Constants.TURRET_LIMIT_BUFFER_DFT);
  private DoublePreferenceConstant p_turretP = new DoublePreferenceConstant("Turret P", Constants.TURRET_P_DFT);
  private DoublePreferenceConstant p_turretI = new DoublePreferenceConstant("Turret I", Constants.TURRET_I_DFT);
  private DoublePreferenceConstant p_turretD = new DoublePreferenceConstant("Turret D", Constants.TURRET_D_DFT);
  private DoublePreferenceConstant p_turretF = new DoublePreferenceConstant("Turret F", Constants.TURRET_F_DFT);
  private DoublePreferenceConstant p_maxVelocity = new DoublePreferenceConstant("Turret Max Velocity", 0);
  private DoublePreferenceConstant p_maxAcceleration = new DoublePreferenceConstant("Turret Max Acceleration", 0);

  // 
  private boolean m_tracking = false;

  /** Creates a new Turret. */
  public Turret() {
    // configure TalonFX
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    config.motionCruiseVelocity = p_maxVelocity.getValue();
    config.motionAcceleration = p_maxAcceleration.getValue();
    config.slot0.kP = p_turretP.getValue();
    config.slot0.kI = p_turretI.getValue();
    config.slot0.kD = p_turretD.getValue();
    config.slot0.kF = p_turretF.getValue();
    config.forwardSoftLimitThreshold = p_forwardLimit.getValue();
    config.forwardSoftLimitEnable = true;
    config.reverseSoftLimitThreshold = p_reverseLimit.getValue();
    config.reverseSoftLimitEnable = true;
    config.peakOutputForward = 1.0;
    config.peakOutputReverse = -1.0;
    config.nominalOutputForward = p_nominalForward.getValue();
    config.nominalOutputReverse = p_nominalReverse.getValue();
    config.neutralDeadband = 0.001;
    m_turret.configAllSettings(config);

    // configure CANCoder
    CANCoderConfiguration encConfig = new CANCoderConfiguration();
    encConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    encConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

    // Other configuration options, with defaults noted
    // velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms
    // velocityMeasurementWindow = 64
    // magnetOffsetDegrees = 0
    // sensorDirection = false
    // sensorCoefficient = 360.0 / 4096.0
    // unitString = "deg"
    // sensorTimeBase = SensorTimeBase.PerSecond

    m_cancoder.configAllSettings(encConfig);

    sync();
  }

  public void sync() {
    // initialize TalonFX to correct absolute position when we wake up
    m_turret.setSelectedSensorPosition(cancoderPostionToFalconPosition(m_cancoder.getAbsolutePosition()));
  }

  public void calibrate() {
    // assume the turret has been physically moved to its center position
    p_zeroPosition.setValue(m_cancoder.getAbsolutePosition());
    sync();
  }

  public void rawMotor(double percentOutput) {
    m_turret.set(TalonFXControlMode.PercentOutput, percentOutput);
  }

  public void goToPosition(double position) {
    m_turret.set(TalonFXControlMode.MotionMagic, position);
  }

  public void goToFacing(double target) {
    goToPosition(turretFacingToEncoderPosition(target));
  }

  public boolean isPositionSafe(double position) {
    return (position < p_forwardLimit.getValue() - p_limitBuffer.getValue()) &&
      (position > p_reverseLimit.getValue() + p_limitBuffer.getValue());
  }

  public boolean isFacingSafe(double degrees) {
    return isPositionSafe(turretFacingToEncoderPosition(degrees));
  }

  public double getPosition() {
    return m_turret.getSelectedSensorPosition();
  }
  
  public double getFacing() {
    return turretEncoderPositionToFacing(getPosition());
  }

  public boolean isSynchronized() {
    return Math.abs(getPosition() - cancoderPostionToFalconPosition(m_cancoder.getAbsolutePosition())) < Constants.TURRET_SYNCRONIZATION_THRESHOLD;
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

  private double cancoderPostionToFalconPosition(double position) {
    return turretFacingToEncoderPosition((position - p_zeroPosition.getValue()) * 
      (Constants.TURRET_CANCODER_GEAR_RATIO/Constants.TURRET_GEAR_RATIO));
  }

  private double falconPositionToCancoderPostion(double position) {
    return turretEncoderPositionToFacing(position) + p_zeroPosition.getValue();
  }

  public double turretEncoderPositionToFacing(double turretPosition) {
    return (turretPosition / Constants.TURRET_COUNTS_PER_REV) * 360.0;
  }

  public double turretFacingToEncoderPosition(double degrees) {
    return (degrees / 360.0) * Constants.TURRET_COUNTS_PER_REV;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret:CANCoder Absolute", m_cancoder.getAbsolutePosition());
    SmartDashboard.putNumber("Turret:CANCoder Position", m_cancoder.getPosition());
    SmartDashboard.putNumber("Turret:CANCoder Turret Facing",  turretEncoderPositionToFacing(cancoderPostionToFalconPosition(m_cancoder.getAbsolutePosition())));
    SmartDashboard.putNumber("Turret:Position", getPosition());
    SmartDashboard.putNumber("Turret:Facing", getFacing());
  }
}
