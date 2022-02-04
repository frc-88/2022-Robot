// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * TODO: haiku
 */

public class Feeder extends SubsystemBase {
  private TalonFX m_feederMotor;
  private DigitalInput m_feederBeambreak;
  private double m_feederMotorSpeed;

  public Feeder(int feederMotorId, int feederSensorId, double feederMotorSpeed) {
    m_feederMotor = new TalonFX(feederMotorId);
    m_feederBeambreak = new DigitalInput(feederSensorId);
    m_feederMotorSpeed = feederMotorSpeed;

    TalonFXConfiguration config = new TalonFXConfiguration();
    m_feederMotor.configAllSettings(config);
  }

  public boolean hasCargo() {
    return m_feederBeambreak.get();
  }

  public void run() {
    m_feederMotor.set(ControlMode.PercentOutput, m_feederMotorSpeed);
  }

  public void reverse() {
    m_feederMotor.set(ControlMode.PercentOutput, -m_feederMotorSpeed);
  }

  public void stop() {
    m_feederMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Feeder:" + m_feederMotor.getDeviceID() + ":hasCargo?", hasCargo());
  }
}
