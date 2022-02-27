// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CargoSource;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;
import frc.robot.util.sensors.SharpIR;

public class Intake extends SubsystemBase implements CargoSource {

  private final WPI_TalonFX m_roller;
  private final WPI_TalonFX m_arm;
  private final SharpIR m_IR;

  private boolean m_isCalibrated = false;

  private DoublePreferenceConstant rollerIntakeSpeed;
  private DoublePreferenceConstant rolleroutgestSpeed;
  private DoublePreferenceConstant armCurrentControlTarget;

  private DoublePreferenceConstant continuousCurrentLimit;
  private DoublePreferenceConstant triggerCurrentLimit;
  private DoublePreferenceConstant triggerDuration;
  private DoublePreferenceConstant armMaxVelocity;
  private DoublePreferenceConstant armMaxAcceleration;
  private PIDPreferenceConstants armMotionMagicPID;
  private PIDPreferenceConstants armCurrentPID;
  private DoublePreferenceConstant armCurrentControlMaxPercent;
  private DoublePreferenceConstant intakseSensorThreshold;

  private static final int MOTION_MAGIC_PID_SLOT = 0;
  private static final int CURRENT_CONTROL_PID_SLOT = 1;

  private static final double CALIBRATION_COLLECT_SIZE = 25;
  private static final double CALIBRATION_TOLERANCE = 0.5;
  private double m_calibrationStartValue = 0;
  private double m_calibrationCollectsDone = 0;

  private static final double ARM_RATIO = 360. / (5. * 5. * (40./32.) * (44./18.) * 2048.); // Motor ticks to actual degrees

  public static final double ARM_STOWED = 93;
  public static final double ARM_DEPLOYED = 0;

  private static final double ARM_SETPOINT_TOLERANCE = 5;

  private State m_state = State.STARTUP;
  private static enum State {
    STARTUP,
    DEPLOYING,
    DEPLOYED_CALIBRATING,
    DEPLOYED,
    STOWING,
    STOWED_CALIBRATING,
    STOWED
  }

  /** Creates a new Intake. */
  public Intake() {
    m_roller = new WPI_TalonFX(Constants.INTAKE_ROLLER_ID, "1");
    m_arm = new WPI_TalonFX(Constants.INTAKE_ARM_ID, "1");
    m_IR = new SharpIR(Constants.INTAKE_IR_ID);

    armCurrentControlTarget = new DoublePreferenceConstant("Intake Arm Current Control Target", 8);
    rollerIntakeSpeed = new DoublePreferenceConstant("Intake Roller Intake Speed", 0.5);
    rolleroutgestSpeed = new DoublePreferenceConstant("Intake Roller Outgest Speed", -0.5);

    continuousCurrentLimit = new DoublePreferenceConstant("Intake Arm Continuous Current", 20);
    triggerCurrentLimit = new DoublePreferenceConstant("Intake Arm Trigger Current", 80);
    triggerDuration = new DoublePreferenceConstant("Intake Arm Trigger Current Duration", 0.002);
    armMaxVelocity = new DoublePreferenceConstant("Intake Arm Max Velocity", 0);
    armMaxAcceleration = new DoublePreferenceConstant("Intake Arm Acceleration", 0);
    armMotionMagicPID = new PIDPreferenceConstants("Intake Arm Motion Magic", 0, 0, 0, 0, 0, 0, 0);
    armCurrentPID = new PIDPreferenceConstants("Intake Arm Current", 0, 0, 0, 0, 0, 0, 0);
    armCurrentControlMaxPercent = new DoublePreferenceConstant("Intake Arm Current Control Max Percent", 0.1);
    intakseSensorThreshold =  new DoublePreferenceConstant("Intake Sensor Threshold", 15.0);


    continuousCurrentLimit.addChangeHandler((Double unused) -> configCurrentLimit());
    triggerCurrentLimit.addChangeHandler((Double unused) -> configCurrentLimit());
    triggerDuration.addChangeHandler((Double unused) -> configCurrentLimit());
    armMaxVelocity.addChangeHandler((Double unused) -> configMotionMagic());
    armMaxAcceleration.addChangeHandler((Double unused) -> configMotionMagic());
    armMotionMagicPID.addChangeHandler((Double unused) -> configMotionMagic());
    armCurrentPID.addChangeHandler((Double unused) -> configCurrentControl());
    armCurrentControlMaxPercent.addChangeHandler((Double unused) -> configCurrentControl());

    m_roller.configFactoryDefault();
    m_arm.configFactoryDefault();

    m_arm.setInverted(InvertType.InvertMotorOutput);

    configCurrentLimit();
    configMotionMagic();
    configCurrentControl();
    
    m_arm.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    m_arm.configForwardSoftLimitThreshold(convertArmPositionToMotor(ARM_STOWED));
    m_arm.configForwardSoftLimitEnable(true);
    m_arm.configReverseSoftLimitThreshold(convertArmPositionToMotor(ARM_DEPLOYED));
    m_arm.configReverseSoftLimitEnable(true);
    m_arm.configNeutralDeadband(0);
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
    m_arm.config_kP(MOTION_MAGIC_PID_SLOT, armMotionMagicPID.getKP().getValue());
    m_arm.config_kI(MOTION_MAGIC_PID_SLOT, armMotionMagicPID.getKI().getValue());
    m_arm.config_kD(MOTION_MAGIC_PID_SLOT, armMotionMagicPID.getKD().getValue());
    m_arm.config_kF(MOTION_MAGIC_PID_SLOT, armMotionMagicPID.getKF().getValue());
    m_arm.config_IntegralZone(MOTION_MAGIC_PID_SLOT, armMotionMagicPID.getIZone().getValue());
    m_arm.configMaxIntegralAccumulator(MOTION_MAGIC_PID_SLOT, armMotionMagicPID.getIMax().getValue());
    m_arm.configClosedLoopPeakOutput(MOTION_MAGIC_PID_SLOT, 1);
  }

  private void configCurrentControl() {
    m_arm.config_kP(CURRENT_CONTROL_PID_SLOT, armCurrentPID.getKP().getValue());
    m_arm.config_kI(CURRENT_CONTROL_PID_SLOT, armCurrentPID.getKI().getValue());
    m_arm.config_kD(CURRENT_CONTROL_PID_SLOT, armCurrentPID.getKD().getValue());
    m_arm.config_kF(CURRENT_CONTROL_PID_SLOT, armCurrentPID.getKF().getValue());
    m_arm.config_IntegralZone(CURRENT_CONTROL_PID_SLOT, armCurrentPID.getIZone().getValue());
    m_arm.configMaxIntegralAccumulator(CURRENT_CONTROL_PID_SLOT, armCurrentPID.getIMax().getValue());
    m_arm.configClosedLoopPeakOutput(CURRENT_CONTROL_PID_SLOT, armCurrentControlMaxPercent.getValue());
  }

  private void enableLimits() {
    m_arm.overrideLimitSwitchesEnable(true);
    m_arm.overrideSoftLimitsEnable(true);
  }

  private void disableLimits() {
    m_arm.overrideLimitSwitchesEnable(false);
    m_arm.overrideSoftLimitsEnable(false);
  }


  public double getArmPosition() {
    return convertMotorPositionToArm(m_arm.getSelectedSensorPosition());
  }

  public double getArmVelocity() {
    return convertMotorVelocityToArm(m_arm.getSelectedSensorVelocity());
  }


  @Override
  public boolean hasCargo() {
    // return m_IR.getDistance() < intakseSensorThreshold.getValue();
    return m_state == State.DEPLOYED || m_state == State.DEPLOYING;
  }

  public boolean isDeployLimitTriggered() {
    return m_arm.isRevLimitSwitchClosed() > 0;
  }


  public void deploy() {
    switch(m_state) {
      case STARTUP:
        m_arm.setSelectedSensorPosition(convertArmPositionToMotor(ARM_STOWED));

        // Intentional fall-through
      
      case DEPLOYING:
      case STOWING:
      case STOWED:
      case STOWED_CALIBRATING:
        enableLimits();
        setArmMotionMagic(ARM_DEPLOYED);

        if (getArmPosition() < ARM_DEPLOYED + ARM_SETPOINT_TOLERANCE || isDeployLimitTriggered()) {
          if (m_isCalibrated) {
            m_state = State.DEPLOYED;
          } else {
            m_calibrationStartValue = getArmPosition();
            m_calibrationCollectsDone = 0;
            m_state = State.DEPLOYED_CALIBRATING;
          }
        } else {
          m_state = State.DEPLOYING;
        }

        break;

      case DEPLOYED_CALIBRATING:
        if (checkCalibration(-1)) {
          m_arm.setSelectedSensorPosition(convertArmPositionToMotor(ARM_DEPLOYED));

          m_isCalibrated = true;
          m_state = State.DEPLOYED;
        } else {
          m_state = State.DEPLOYED_CALIBRATING;
        }

        // Intentional Fall-through

      case DEPLOYED:
        disableLimits();
        setArmCurrentControl(-1);

        break;
    }
  }

  public void stow() {
    switch(m_state) {
      case STARTUP:
        m_arm.setSelectedSensorPosition(convertArmPositionToMotor(ARM_STOWED));

        // Intentional fall-through
      
      case STOWING:
      case DEPLOYING:
      case DEPLOYED:
      case DEPLOYED_CALIBRATING:
        enableLimits();
        setArmMotionMagic(ARM_STOWED);

        if (getArmPosition() > ARM_STOWED - ARM_SETPOINT_TOLERANCE) {
          if (m_isCalibrated) {
            m_state = State.STOWED;
          } else {
            m_calibrationStartValue = getArmPosition();
            m_calibrationCollectsDone = 0;
            m_state = State.STOWED_CALIBRATING;
          }
        } else {
          m_state = State.STOWING;
        }

        break;

      case STOWED_CALIBRATING:
        if (checkCalibration(1)) {
          m_arm.setSelectedSensorPosition(convertArmPositionToMotor(ARM_STOWED));

          m_isCalibrated = true;
          m_state = State.STOWED;
        } else {
          m_state = State.STOWED_CALIBRATING;
        }

        // Intentional Fall-through

      case STOWED:
        disableLimits();
        setArmCurrentControl(1);

        break;
    }
  }

  private void setArmMotionMagic(double position) {
    m_arm.selectProfileSlot(MOTION_MAGIC_PID_SLOT, 0);
    m_arm.set(TalonFXControlMode.MotionMagic, convertArmPositionToMotor(position));
  }

  private void setArmCurrentControl(int direction) {
    m_arm.selectProfileSlot(CURRENT_CONTROL_PID_SLOT, 0);
    m_arm.set(TalonFXControlMode.Current, armCurrentControlTarget.getValue() * direction);
  }

  private boolean checkCalibration(int movementDirection) {
    double currentPosition = getArmPosition();
    if (Math.abs(currentPosition + (m_calibrationStartValue * movementDirection)) < CALIBRATION_TOLERANCE) {
      m_calibrationCollectsDone++;
    } else {
      m_calibrationCollectsDone = 0;
      m_calibrationStartValue = currentPosition;
    }

    return m_calibrationCollectsDone >= CALIBRATION_COLLECT_SIZE;
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
  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Arm Position", getArmPosition());
    SmartDashboard.putNumber("Intake Arm Velocity", getArmVelocity());
    SmartDashboard.putNumber("Intake Arm Current", m_arm.getSupplyCurrent());
    SmartDashboard.putBoolean("Intake Arm Calibrated", m_isCalibrated);
    SmartDashboard.putBoolean("Intake Arm Deploy Limit", isDeployLimitTriggered());

    SmartDashboard.putNumber("Intake Roller Current", m_arm.getSupplyCurrent());

    SmartDashboard.putBoolean("Intake Has Cargo", hasCargo());
    SmartDashboard.putNumber("Intake Sensor Distance", m_IR.getDistance());

    SmartDashboard.putString("Intake State", m_state.toString());
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