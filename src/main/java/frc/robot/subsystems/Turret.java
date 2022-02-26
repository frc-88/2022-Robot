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
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class Turret extends SubsystemBase {
  private TalonFX m_turret = new TalonFX(Constants.TURRET_MOTOR_ID, "1");
  private CANCoder m_cancoder = new CANCoder(Constants.TURRET_CANCODER_ID, "1");

  // Preferences
  private DoublePreferenceConstant p_zeroPosition = new DoublePreferenceConstant("Turret Zero", 0.0);
  private DoublePreferenceConstant p_limitBuffer = new DoublePreferenceConstant("Turret Limit Buffer", 0.0);
  private DoublePreferenceConstant p_syncThreshold = new DoublePreferenceConstant("Turret Sync Threshold", 0.0);
  private PIDPreferenceConstants p_turretPID = new PIDPreferenceConstants("Turret PID", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  private DoublePreferenceConstant p_maxVelocity = new DoublePreferenceConstant("Turret Max Velocity", 0);
  private DoublePreferenceConstant p_maxAcceleration = new DoublePreferenceConstant("Turret Max Acceleration", 0);
  private DoublePreferenceConstant p_nominalForward = new DoublePreferenceConstant("Turret Nominal Forward", 0.0);
  private DoublePreferenceConstant p_nominalReverse = new DoublePreferenceConstant("Turret Nominal Reverse", 0.0);
  private DoublePreferenceConstant p_forwardLimit = new DoublePreferenceConstant("Turret Forward Limit", 0.0);
  private DoublePreferenceConstant p_reverseLimit = new DoublePreferenceConstant("Turret Reverse Limit", 0.0);
 
  // 
  private boolean m_tracking = false;

  /** Creates a new Turret. */
  public Turret() {
    configureFalcon();
    configureCANCoder();

    p_turretPID.addChangeHandler((Double unused) -> configureFalcon());
    p_maxVelocity.addChangeHandler((Double unused) -> configureFalcon());
    p_maxAcceleration.addChangeHandler((Double unused) -> configureFalcon());
    p_nominalForward.addChangeHandler((Double unused) -> configureFalcon());
    p_nominalReverse.addChangeHandler((Double unused) -> configureFalcon());
    p_forwardLimit.addChangeHandler((Double unused) -> configureFalcon());
    p_reverseLimit.addChangeHandler((Double unused) -> configureFalcon());

    // initialize Falcon to correct position when we wake up based on CANcoder absolute position
    sync();
  }

  private void configureFalcon() {
    // configure TalonFX
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    config.motionCruiseVelocity = p_maxVelocity.getValue();
    config.motionAcceleration = p_maxAcceleration.getValue();
    config.slot0.kP = p_turretPID.getKP().getValue();
    config.slot0.kI = p_turretPID.getKI().getValue();
    config.slot0.kD = p_turretPID.getKD().getValue();
    config.slot0.kF = p_turretPID.getKF().getValue();
    config.slot0.integralZone = p_turretPID.getIZone().getValue();
    config.slot0.maxIntegralAccumulator = p_turretPID.getIMax().getValue();
    config.forwardSoftLimitThreshold = turretFacingToEncoderPosition(p_forwardLimit.getValue());
    config.forwardSoftLimitEnable = true;
    config.reverseSoftLimitThreshold = turretFacingToEncoderPosition(p_reverseLimit.getValue());
    config.reverseSoftLimitEnable = true;
    config.peakOutputForward = 1.0;
    config.peakOutputReverse = -1.0;
    config.nominalOutputForward = p_nominalForward.getValue();
    config.nominalOutputReverse = p_nominalReverse.getValue();
    config.neutralDeadband = 0.001;
    m_turret.configAllSettings(config);
  }

  private void configureCANCoder() {
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
  }

  public void sync() {
    m_turret.setSelectedSensorPosition(cancoderPostionToFalconPosition(m_cancoder.getAbsolutePosition()));
  }

  public void calibrate() {
    // This is only necessary if the CANcoder is moved or adjusted.
    // The turret must be physically moved to its center position.
    // WARNING - doing this when the turret isn't in the "zero"
    // position could cause the turret to move to unsafe positions.
    p_zeroPosition.setValue(m_cancoder.getAbsolutePosition());
    sync();
  }

  public void setPercentOutput(double percentOutput) {
    m_turret.set(TalonFXControlMode.PercentOutput, percentOutput);
  }

  public void goToFacing(double target) {
    goToPosition(turretFacingToEncoderPosition(target));
  }

  public boolean isFacingSafe(double degrees) {
    return isPositionSafe(turretFacingToEncoderPosition(degrees));
  }

  public double getFacing() {
    return turretEncoderPositionToFacing(getPosition());
  }

  public boolean isSynchronized() {
    return Math.abs(getFacing() - 
      turretEncoderPositionToFacing(cancoderPostionToFalconPosition(m_cancoder.getAbsolutePosition()))) < p_syncThreshold.getValue();
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

  private double getPosition() {
    return m_turret.getSelectedSensorPosition();
  }
  
  private void goToPosition(double position) {
    m_turret.set(TalonFXControlMode.MotionMagic, position);
  }

  private boolean isPositionSafe(double position) {
    return (position < turretFacingToEncoderPosition(p_forwardLimit.getValue() - p_limitBuffer.getValue())) &&
      (position > turretFacingToEncoderPosition(p_reverseLimit.getValue() + p_limitBuffer.getValue()));
  }

  private double cancoderPostionToFalconPosition(double position) {
    return turretFacingToEncoderPosition((position - p_zeroPosition.getValue()) * 
      (Constants.TURRET_CANCODER_GEAR_RATIO/Constants.TURRET_GEAR_RATIO));
  }

  private double falconPositionToCancoderPostion(double position) {
    return turretEncoderPositionToFacing(position) + p_zeroPosition.getValue();
  }

  private double turretEncoderPositionToFacing(double turretPosition) {
    return (turretPosition / Constants.TURRET_COUNTS_PER_REV) * 360.0;
  }

  private double turretFacingToEncoderPosition(double degrees) {
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
    SmartDashboard.putBoolean("Turret:Synchonized", isSynchronized());
    SmartDashboard.putBoolean("Turret:Tracking", isTracking());
    SmartDashboard.putBoolean("Turret:Safe", isPositionSafe(getPosition()));
  }
}
