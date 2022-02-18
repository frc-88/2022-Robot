// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CargoSource;
import frc.robot.util.CargoTarget;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

/*
 * feed me some cargo
 * then I can pass it along
 * I can cargolize!
 */

public class Feeder extends SubsystemBase implements CargoSource, CargoTarget {
  private TalonFX m_feederMotor;
  private DigitalInput m_feederBeambreak;
  private DoublePreferenceConstant p_feederMotorSpeed;

  public Feeder(int feederMotorId, int feederSensorId, DoublePreferenceConstant feederMotorSpeedPref) {
    m_feederMotor = new TalonFX(feederMotorId);
    m_feederBeambreak = new DigitalInput(feederSensorId);
    p_feederMotorSpeed = feederMotorSpeedPref;

    // basic config, stagger status frames we don't care about
    TalonFXConfiguration config = new TalonFXConfiguration();
    m_feederMotor.configAllSettings(config);
    setStatusFrames();
  }

  public void run() {
    m_feederMotor.set(ControlMode.PercentOutput, p_feederMotorSpeed.getValue());
  }

  public void reverse() {
    m_feederMotor.set(ControlMode.PercentOutput, -p_feederMotorSpeed.getValue());
  }

  public void stop() {
    m_feederMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public boolean hasCargo() {
    return !m_feederBeambreak.get();
  }

  @Override
  public boolean wantsCargo() {
    return !hasCargo();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Feeder:" + m_feederMotor.getDeviceID() + ":hasCargo?", hasCargo());

    if (m_feederMotor.hasResetOccurred()) {
      setStatusFrames();
    }
  }

  private void setStatusFrames() {
    m_feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 5000);
    m_feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5000);
    m_feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 5000);
    m_feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 5000);
    m_feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 5000);
    m_feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 5000);
    m_feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5000);
    m_feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 5000);
    m_feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 5000);
    m_feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 5000);
  }
}
