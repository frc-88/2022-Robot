// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CargoTarget;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.sensors.Limelight;

public class Shooter extends SubsystemBase implements CargoTarget {
  private TalonFX m_flywheel = new TalonFX(Constants.SHOOTER_FLYWHEEL_ID);
  private TalonFX m_hood = new TalonFX(Constants.SHOOTER_HOOD_ID);
  private Limelight m_limelight;

  // Preferences
  private DoublePreferenceConstant p_shooterP = new DoublePreferenceConstant("Shooter P", Constants.SHOOTER_P_DFT);
  private DoublePreferenceConstant p_shooterI = new DoublePreferenceConstant("Shooter I", Constants.SHOOTER_I_DFT);
  private DoublePreferenceConstant p_shooterD = new DoublePreferenceConstant("Shooter D", Constants.SHOOTER_D_DFT);
  private DoublePreferenceConstant p_shooterF = new DoublePreferenceConstant("Shooter F", Constants.SHOOTER_F_DFT);
  private DoublePreferenceConstant p_hoodSpeed = new DoublePreferenceConstant("Hood Speed", Constants.SHOOTER_HOOD_SPEED_DFT);

  /** Creates a new Shooter. */
  public Shooter(Limelight limelight) {
    m_limelight = limelight;
    configureFlywheel();
    configureHood();
    setStatusFrames();
  }

  private void configureFlywheel() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    config.slot0.kP = p_shooterP.getValue();
    config.slot0.kI = p_shooterI.getValue();
    config.slot0.kD = p_shooterD.getValue();
    config.slot0.kF = p_shooterF.getValue();
    config.neutralDeadband = 0.001;
    config.peakOutputForward = 1.0;
    config.peakOutputReverse = -1.0;
    config.nominalOutputForward = 0;
    config.nominalOutputReverse = 0;

    m_flywheel.configAllSettings(config);
  }

  private void configureHood() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    config.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 20, 25, 1.0);
    config.peakOutputForward = 1.0;
    config.peakOutputReverse = -1.0;
    config.nominalOutputForward = 0;
    config.nominalOutputReverse = 0;
    m_hood.configAllSettings(config);
  }

  public void setFlywheelSpeed(double speed) {
    m_flywheel.set(TalonFXControlMode.Velocity, speed);
  }

  public void setFlywheelRaw(double percentOutput) {
    m_flywheel.set(TalonFXControlMode.PercentOutput, percentOutput);
  }

  public boolean onTarget() {
    return Math.abs(m_flywheel.getClosedLoopError()) < Constants.SHOOTER_FLYWHEEL_ERROR_THRESHOLD;
  }

  public void raiseHood() {
    m_hood.set(TalonFXControlMode.PercentOutput, p_hoodSpeed.getValue());
  }

  public void lowerHood() {
    m_hood.set(TalonFXControlMode.PercentOutput, -p_hoodSpeed.getValue());
  }

  @Override
  public boolean wantsCargo() {
    // TODO
    return false && onTarget() && m_limelight.hasTarget() 
      && (Math.abs(m_limelight.getTargetHorizontalOffsetAngle()) < Constants.SHOOTER_LIMELIGHT_THRESHOLD);
  }
  
  @Override
  public void periodic() {
    if (m_flywheel.hasResetOccurred() || m_hood.hasResetOccurred()) {
      setStatusFrames();
    }
  }

  public void setStatusFrames() {
    m_flywheel.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
    m_flywheel.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 5000);
    m_flywheel.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 5000);
    m_flywheel.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 5000);
    m_flywheel.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 5000);
    m_flywheel.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5000);
    m_flywheel.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 5000);

    m_hood.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
    m_hood.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 5000);
    m_hood.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 5000);
    m_hood.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 5000);
    m_hood.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 5000);
    m_hood.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5000);
    m_hood.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 5000);
  }
}
