// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  private TalonFX m_turret;

  /** Creates a new Turret. */
  public Turret() {
    m_turret = new TalonFX(Constants.TURRET_MOTOR_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

