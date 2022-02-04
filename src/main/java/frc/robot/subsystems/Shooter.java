// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private TalonFX m_flywheel, m_hood;

  /** Creates a new Shooter. */
  public Shooter(int flywheelID, int hoodID) {
    m_flywheel = new TalonFX(flywheelID);
    m_hood = new TalonFX(hoodID);

    TalonFXConfiguration flywheelCfg = new TalonFXConfiguration();
    TalonFXConfiguration hoodCfg = new TalonFXConfiguration();

    m_flywheel.configAllSettings(flywheelCfg);
    m_hood.configAllSettings(hoodCfg);
  }

  public void setFlywheelSpeed(double speed) {
    m_flywheel.set(TalonFXControlMode.Velocity, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
