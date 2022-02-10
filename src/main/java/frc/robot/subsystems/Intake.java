// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private final WPI_TalonFX m_roller;
  private final WPI_TalonFX m_arm;

  /** Creates a new Intake. */
  public Intake() {
    m_roller = new WPI_TalonFX(Constants.INTAKE_ROLLER_ID);
    m_arm = new WPI_TalonFX(Constants.INTAKE_ARM_ID);

    m_roller.configFactoryDefault();
    m_arm.configFactoryDefault();
  }

  public void setRollerPercent(double percent) {
    m_roller.set(TalonFXControlMode.PercentOutput, percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //adding this so commit can work (this can be deleted)
  }
}