// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class Intake extends SubsystemBase {

  private final WPI_TalonFX m_roller;
  private final WPI_TalonFX m_arm;

  private boolean isGuessPositionSet = false;
  private boolean isCalibrated = false;

  private DoublePreferenceConstant rollerIntakeSpeed;
  private DoublePreferenceConstant rolleroutgestSpeed;

  private DoublePreferenceConstant continuousCurrentLimit;
  private DoublePreferenceConstant triggerCurrentLimit;
  private DoublePreferenceConstant triggerDuration;
  private DoublePreferenceConstant armMaxVelocity;
  private DoublePreferenceConstant armMaxAcceleration;
  private PIDPreferenceConstants armPID;

  private static final double ARM_RATIO = 360. / (5 * 5 * (40/32) * (44/18) * 2048); // Motor ticks to actual degrees

  private static final double ARM_STOWED = 90;
  private static final double ARM_DEPLOYED = 0;

  /** Creates a new Intake. */
  public Intake() {
    rollerIntakeSpeed = new DoublePreferenceConstant("Intake Roller Intake Speed", 0.5);
    rolleroutgestSpeed = new DoublePreferenceConstant("Intake Roller Outgest Speed", -0.5);

    continuousCurrentLimit = new DoublePreferenceConstant("Intake Arm Continuous Current", 20);
    triggerCurrentLimit = new DoublePreferenceConstant("Intake Arm Trigger Current", 80);
    triggerDuration = new DoublePreferenceConstant("Intake Arm Trigger Current Duration", 0.002);
    armMaxVelocity = new DoublePreferenceConstant("Intake Arm Max Velocity", 0);
    armMaxAcceleration = new DoublePreferenceConstant("Intake Arm Acceleration", 0);
    armPID = new PIDPreferenceConstants("Intake Arm", 0, 0, 0, 0, 0, 0, 0);



    m_roller = new WPI_TalonFX(Constants.INTAKE_ROLLER_ID);
    m_arm = new WPI_TalonFX(Constants.INTAKE_ARM_ID);

    m_roller.configFactoryDefault();
    m_arm.configFactoryDefault();

    configCurrentLimit();
    configMotionMagic();
    
    m_arm.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    m_arm.configForwardSoftLimitThreshold(convertArmPositionToMotor(ARM_STOWED));
    m_arm.configForwardSoftLimitEnable(true);
    m_arm.configReverseSoftLimitThreshold(convertArmPositionToMotor(ARM_DEPLOYED));
    m_arm.configReverseSoftLimitEnable(true);
  }

  private void configCurrentLimit() {
    StatorCurrentLimitConfiguration armCurrentLimit = new StatorCurrentLimitConfiguration();
    armCurrentLimit.enable = true;
    armCurrentLimit.currentLimit = continuousCurrentLimit.getValue();
    armCurrentLimit.triggerThresholdCurrent = triggerCurrentLimit.getValue();
    armCurrentLimit.triggerThresholdTime = triggerDuration.getValue();
    m_arm.configStatorCurrentLimit(armCurrentLimit);
  }

  private void configMotionMagic() {
    m_arm.configMotionCruiseVelocity(convertArmVelocityToMotor(armMaxVelocity.getValue()));
    m_arm.configMotionAcceleration(convertArmVelocityToMotor(armMaxAcceleration.getValue()));
    m_arm.config_kP(0, armPID.getKP().getValue());
    m_arm.config_kI(0, armPID.getKI().getValue());
    m_arm.config_kD(0, armPID.getKD().getValue());
    m_arm.config_kF(0, armPID.getKF().getValue());
    m_arm.config_IntegralZone(0, armPID.getIZone().getValue());
    m_arm.configMaxIntegralAccumulator(0, armPID.getIMax().getValue());
  }

  private void enableLimits() {
    m_arm.overrideLimitSwitchesEnable(true);
    m_arm.overrideSoftLimitsEnable(true);
  }

  private void disableLimits() {
    m_arm.overrideLimitSwitchesEnable(false);
    m_arm.overrideSoftLimitsEnable(false);
  }


  public void rollerIntake() {
    m_roller.set(TalonFXControlMode.PercentOutput, rollerIntakeSpeed.getValue());
  }

  public void rollerStop() {
    m_roller.stopMotor();
  }

  public void rollerOutgest() {
    m_roller.set(TalonFXControlMode.PercentOutput, rolleroutgestSpeed.getValue());
  }


  public double getArmPosition() {
    return convertMotorPositionToArm(m_arm.getSelectedSensorPosition());
  }

  public double getArmVelocity() {
    return convertMotorVelocityToArm(m_arm.getSelectedSensorVelocity());
  }
  

  @Override
  public void periodic() {
    if (!isGuessPositionSet && DriverStation.isEnabled()) {
      m_arm.setSelectedSensorPosition(convertArmPositionToMotor(ARM_STOWED));
      isGuessPositionSet = true;
    }

    SmartDashboard.putNumber("Intake Arm Position", getArmPosition());
    SmartDashboard.putNumber("Intake Arm Velocity", getArmVelocity());
    SmartDashboard.putNumber("Intake Arm Current", m_arm.getSupplyCurrent());
    SmartDashboard.putBoolean("Intake Arm Calibrated", isCalibrated);
    SmartDashboard.putBoolean("Intake Arm Reverse Limit", m_arm.isRevLimitSwitchClosed() > 0);

    SmartDashboard.putNumber("Intake Roller Current", m_arm.getSupplyCurrent());
  }

  private double convertMotorPositionToArm(double motorPosition) {
    return motorPosition * ARM_RATIO;
  }

  private double convertMotorVelocityToArm(double motorVelocity) {
    return convertMotorPositionToArm(motorVelocity) * 10.;
  }

  private double convertArmPositionToMotor(double armPosition) {
    return armPosition / ARM_RATIO;
  }

  private double convertArmVelocityToMotor(double armVelocity) {
    return convertArmPositionToMotor(armVelocity) * 0.1;
  }
}