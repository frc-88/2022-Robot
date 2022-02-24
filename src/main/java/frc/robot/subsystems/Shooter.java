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

  // TODO - enter real values for this table
  private final ValueInterpolator distanceToSpeedInterpolator = new ValueInterpolator(
    new ValueInterpolator.ValuePair(127, 5100),
    new ValueInterpolator.ValuePair(173, 4600),
    new ValueInterpolator.ValuePair(213, 4300),
    new ValueInterpolator.ValuePair(245, 4275),
    new ValueInterpolator.ValuePair(294, 4200)
  );

  // Preferences
  private DoublePreferenceConstant p_continuousCurrentLimit = new DoublePreferenceConstant("Hood Continuous Current", 10);
  private DoublePreferenceConstant p_triggerCurrentLimit = new DoublePreferenceConstant("Hood Trigger Current", 80);
  private DoublePreferenceConstant p_triggerDuration = new DoublePreferenceConstant("Hood Trigger Current Duration", 0.002);
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
    config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    config.statorCurrLimit = new StatorCurrentLimitConfiguration(true, p_continuousCurrentLimit.getValue(),
        p_triggerCurrentLimit.getValue(), p_triggerDuration.getValue());
    m_hood.configAllSettings(config);
    m_hood.setInverted(InvertType.InvertMotorOutput);
  }

  public void setFlywheelSpeed(double speed) {
    m_flywheel.set(TalonFXControlMode.Velocity, speed);
  }

  public void setFlywheelRaw(double percentOutput) {
    m_flywheel.set(TalonFXControlMode.PercentOutput, percentOutput);
  }

  public void setFlywheelSpeedFromLimelight() {
    if (m_limelight.hasTarget()) {
      m_flywheel.set(TalonFXControlMode.Velocity, distanceToSpeedInterpolator.getInterpolatedValue(m_limelight.calcDistanceToTarget()));
    }
  }

  public boolean onTarget() {
    return Math.abs(m_flywheel.getClosedLoopError()) < p_flywheelPID.getTolerance().getValue();
  }

  public void raiseHood() {
    m_hood.set(TalonFXControlMode.PercentOutput, p_hoodSpeed.getValue());
    m_limelight.setHood(true);
  }

  public void lowerHood() {
    m_hood.set(TalonFXControlMode.PercentOutput, -p_hoodSpeed.getValue());
    m_limelight.setHood(false);
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Flywheel Position", m_flywheel.getSelectedSensorPosition());
    SmartDashboard.putNumber("Shooter Flywheel Velocity", m_flywheel.getSelectedSensorVelocity());
    SmartDashboard.putBoolean("Shooter Flywheel On Target", onTarget());
  }
}
