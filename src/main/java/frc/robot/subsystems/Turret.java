// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  private TalonFX m_turret;
  private CANCoder m_encoder;

  /** Creates a new Turret. */
  public Turret() {
    m_turret = new TalonFX(Constants.TURRET_MOTOR_ID);
    m_encoder = new CANCoder(Constants.TURRET_ENCODER_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    config.peakOutputForward = 1.0;
    config.peakOutputReverse = -1.0;
    config.nominalOutputForward = 0;
    config.nominalOutputReverse = 0;
    config.neutralDeadband = 0.001;
    // config.slot0.kP = 0.00000;
    // config.slot0.kI = 0.00000;
    // config.slot0.kD = 0.00000;
    // config.slot0.kF = 1.00000;

    m_turret.configAllSettings(config);

    // TODO = use CANcoder to initialize TalonFX integrated sensor position
}

  public void rawMotor(double percentOutput) {
    m_turret.set(TalonFXControlMode.PercentOutput, percentOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

