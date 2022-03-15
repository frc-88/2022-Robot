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
import frc.robot.util.CargoSource;
import frc.robot.util.CargoTarget;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

/*
 * feed me some cargo
 * then I can pass it along
 * I can cargolize!
 */

public class Feeder extends SubsystemBase implements CargoSource, CargoTarget {
  private String m_feederName;
  private TalonFX m_feederMotor;
  private DigitalInput m_feederBeambreak;
  private DoublePreferenceConstant p_feederInSpeed;
  private DoublePreferenceConstant p_feederOutSpeed;
  private DoublePreferenceConstant p_feederIdleSpeed;

  public Feeder(String feederName, int feederMotorId, int feederSensorId, DoublePreferenceConstant feederInSpeedPref, DoublePreferenceConstant feederOutSpeedPref, DoublePreferenceConstant feederIdleSpeedPref) {
    m_feederName = feederName;
    m_feederMotor = new TalonFX(feederMotorId, "1");
    m_feederBeambreak = new DigitalInput(feederSensorId);
    p_feederInSpeed = feederInSpeedPref;
    p_feederOutSpeed = feederOutSpeedPref;

    TalonFXConfiguration config = new TalonFXConfiguration();
    m_feederMotor.configAllSettings(config);
  }

  public void run() {
    m_feederMotor.set(ControlMode.PercentOutput, hasCargo()?p_feederOutSpeed.getValue():p_feederInSpeed.getValue());
  }

  public void reverse() {
    m_feederMotor.set(ControlMode.PercentOutput, -(hasCargo()?p_feederOutSpeed.getValue():p_feederInSpeed.getValue()));
  }

  public void stop() {
    m_feederMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void idle() {
    m_feederMotor.set(ControlMode.PercentOutput, hasCargo()?0.0:p_feederIdleSpeed.getValue());
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
    SmartDashboard.putBoolean(m_feederName + ":hasCargo?", hasCargo());
  }
}
