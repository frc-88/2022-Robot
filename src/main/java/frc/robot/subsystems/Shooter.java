// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CargoTarget;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.sensors.Limelight;
import frc.robot.util.sensors.REVColorSensor;

public class Shooter extends SubsystemBase implements CargoTarget {
  private TalonFX m_flywheel = new TalonFX(Constants.SHOOTER_FLYWHEEL_ID);
  private TalonFX m_hood = new TalonFX(Constants.SHOOTER_HOOD_ID);
  private Turret m_turret;

  // Preferences
  private DoublePreferenceConstant p_shooterP = new DoublePreferenceConstant("Shooter P", Constants.SHOOTER_DEFAULT_P);
  private DoublePreferenceConstant p_shooterI = new DoublePreferenceConstant("Shooter I", Constants.SHOOTER_DEFAULT_I);
  private DoublePreferenceConstant p_shooterD = new DoublePreferenceConstant("Shooter D", Constants.SHOOTER_DEFAULT_D);
  private DoublePreferenceConstant p_shooterF = new DoublePreferenceConstant("Shooter F", Constants.SHOOTER_DEFAULT_F);

  /** Creates a new Shooter. */
  public Shooter(Turret turret) {
    m_turret = turret;
    configureFlywheel();

    TalonFXConfiguration hoodCfg = new TalonFXConfiguration();
    m_hood.configAllSettings(hoodCfg);
  }

  private void configureFlywheel() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    config.slot0.kP = p_shooterP.getValue();
    config.slot0.kI = p_shooterI.getValue();
    config.slot0.kD = p_shooterD.getValue();
    config.slot0.kF = p_shooterF.getValue();
    config.neutralDeadband = 0.001;
    m_flywheel.configAllSettings(config);
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

  @Override
  public boolean wantsCargo() {
    return onTarget(); // && m_turret.onTarget();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
