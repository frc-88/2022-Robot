// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.NumberCache;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class Intake extends SubsystemBase {

  private final PWMTalonFX m_roller;
  private final WPI_TalonFX m_arm;

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
  private DoublePreferenceConstant armArbitraryF;

  private static final int MOTION_MAGIC_PID_SLOT = 0;
  private static final int CURRENT_CONTROL_PID_SLOT = 1;

  private static final double CALIBRATION_COLLECT_SIZE = 50;
  private static final double CALIBRATION_TOLERANCE = 1;
  private double m_calibrationStartValue = 0;
  private double m_calibrationCollectsDone = 0;

  private static final double ARM_RATIO = 360. / ((60./12.) * (44./18.) * 2048.); // Motor ticks to actual degrees

  private static final double ARM_STARTUP_POSITION = 94.;

  public boolean m_armStowCalibrated = false;
  public double m_armStowed = 90;
  public boolean m_armDeployCalibrated = false;
  public double m_armDeployed = 0;

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
    m_roller = new PWMTalonFX(Constants.INTAKE_ROLLER_ID);
    m_arm = new WPI_TalonFX(Constants.INTAKE_ARM_ID, "1");

    armCurrentControlTarget = new DoublePreferenceConstant("Intake Arm Current Control Target", 2);
    rollerIntakeSpeed = new DoublePreferenceConstant("Intake Roller Intake Speed", 0.5);
    rolleroutgestSpeed = new DoublePreferenceConstant("Intake Roller Outgest Speed", -0.5);

    continuousCurrentLimit = new DoublePreferenceConstant("Intake Arm Continuous Current", 15);
    triggerCurrentLimit = new DoublePreferenceConstant("Intake Arm Trigger Current", 80);
    triggerDuration = new DoublePreferenceConstant("Intake Arm Trigger Current Duration", 0.002);
    armMaxVelocity = new DoublePreferenceConstant("Intake Arm Max Velocity", 0);
    armMaxAcceleration = new DoublePreferenceConstant("Intake Arm Acceleration", 0);
    armMotionMagicPID = new PIDPreferenceConstants("Intake Arm Motion Magic", 0, 0, 0, 0, 0, 0, 0);
    armCurrentPID = new PIDPreferenceConstants("Intake Arm Current", 0, 0, 0, 0, 0, 0, 0);
    armCurrentControlMaxPercent = new DoublePreferenceConstant("Intake Arm Current Control Max Percent", 0.1);
    armArbitraryF = new DoublePreferenceConstant("Intake Arbitrary F", 0.0);


    continuousCurrentLimit.addChangeHandler((Double unused) -> configCurrentLimit());
    triggerCurrentLimit.addChangeHandler((Double unused) -> configCurrentLimit());
    triggerDuration.addChangeHandler((Double unused) -> configCurrentLimit());
    armMaxVelocity.addChangeHandler((Double unused) -> configMotionMagic());
    armMaxAcceleration.addChangeHandler((Double unused) -> configMotionMagic());
    armMotionMagicPID.addChangeHandler((Double unused) -> configMotionMagic());
    armCurrentPID.addChangeHandler((Double unused) -> configCurrentControl());
    armCurrentControlMaxPercent.addChangeHandler((Double unused) -> configCurrentControl());
    m_arm.configFactoryDefault();

    m_arm.setInverted(InvertType.InvertMotorOutput);

    configCurrentLimit();
    configMotionMagic();
    configCurrentControl();
    
    m_arm.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    m_arm.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    m_arm.configForwardSoftLimitThreshold(convertArmPositionToMotor(m_armStowed));
    m_arm.configForwardSoftLimitEnable(false);
    m_arm.configReverseSoftLimitThreshold(convertArmPositionToMotor(m_armDeployed));
    m_arm.configReverseSoftLimitEnable(false);
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
    m_arm.configMotionSCurveStrength(6);
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
    if (NumberCache.hasValue("Intake Arm Position")) {
      return NumberCache.getValue("Intake Arm Position");
    }
    return NumberCache.pushValue("Intake Arm Position", convertMotorPositionToArm(m_arm.getSelectedSensorPosition()));
  }

  public double getArmVelocity() {
    return convertMotorVelocityToArm(m_arm.getSelectedSensorVelocity());
  }

  public boolean isDeployLimitTriggered() {
    return m_arm.isRevLimitSwitchClosed() > 0;
  }

  public boolean isStowLimitTriggered() {
    return m_arm.isFwdLimitSwitchClosed() > 0;
  }

  public void deploy() {
    switch(m_state) {
      case STARTUP:
        m_arm.setSelectedSensorPosition(convertArmPositionToMotor(ARM_STARTUP_POSITION));

        // Intentional fall-through
      
      case DEPLOYING:
      case STOWING:
      case STOWED:
      case STOWED_CALIBRATING:

        if (m_armDeployCalibrated) {
            enableLimits();
            setArmMotionMagic(m_armDeployed);

            if (getArmPosition() < m_armDeployed + ARM_SETPOINT_TOLERANCE) {
                m_state = State.DEPLOYED;
            } else {
                m_state = State.DEPLOYING;
            }

            break;

        } else {
            m_calibrationStartValue = getArmPosition();
            m_calibrationCollectsDone = 0;

            // Intentional fall-through
        }

      case DEPLOYED_CALIBRATING:
        enableLimits();
        setArmMotionMagic(m_armDeployed);

        if (checkCalibration()) {
          m_armDeployed = getArmPosition();
          if (!m_armStowCalibrated) {
              m_armStowed = m_armDeployed + ARM_STARTUP_POSITION;
          }
          m_arm.configReverseSoftLimitThreshold(convertArmPositionToMotor(m_armDeployed));

          m_armDeployCalibrated = true;
          m_state = State.DEPLOYED;
        } else {
          m_state = State.DEPLOYED_CALIBRATING;
        }

        break;

      case DEPLOYED:
        disableLimits();
        setArmCurrentControl(-armCurrentControlTarget.getValue());

        break;
    }
  }

  public void stow() {
    switch(m_state) {
      case STARTUP:
        m_arm.setSelectedSensorPosition(convertArmPositionToMotor(ARM_STARTUP_POSITION));

        // Intentional fall-through
      
      case STOWING:
      case DEPLOYING:
      case DEPLOYED:
      case DEPLOYED_CALIBRATING:

        if (m_armStowCalibrated) {
            enableLimits();
            setArmMotionMagic(m_armStowed - 10);

            if (getArmPosition() > m_armStowed - ARM_SETPOINT_TOLERANCE) {
                m_state = State.STOWED;
            } else {
                m_state = State.STOWING;
            }

            break;

        } else {
            m_calibrationStartValue = getArmPosition();
            m_calibrationCollectsDone = 0;

            // Intentional fall-through
        }

      case STOWED_CALIBRATING:
        disableLimits();
        setArmMotionMagic(m_armStowed);

        if (checkCalibration()) {
            m_armStowed = getArmPosition();
            m_arm.configForwardSoftLimitThreshold(convertArmPositionToMotor(m_armStowed));

            m_armStowCalibrated = true;
            m_state = State.STOWED;
        } else {
            m_state = State.STOWED_CALIBRATING;
        }

        break;

      case STOWED:
        disableLimits();
        setArmCurrentControl(armCurrentControlTarget.getValue());

        break;
    }
  }

  private void setArmMotionMagic(double position) {
    m_arm.selectProfileSlot(MOTION_MAGIC_PID_SLOT, 0);
    m_arm.set(TalonFXControlMode.MotionMagic, convertArmPositionToMotor(position), DemandType.ArbitraryFeedForward, armArbitraryF.getValue() * Math.cos(Math.toRadians(position)));
  }

  private void setArmCurrentControl(double current) {
    m_arm.selectProfileSlot(CURRENT_CONTROL_PID_SLOT, 0);
    m_arm.set(TalonFXControlMode.Current, current);
  }

  private boolean checkCalibration() {
    double currentPosition = getArmPosition();
    if (Math.abs(currentPosition - m_calibrationStartValue) < CALIBRATION_TOLERANCE) {
      m_calibrationCollectsDone++;
    } else {
      m_calibrationCollectsDone = 0;
      m_calibrationStartValue = currentPosition;
    }

    return m_calibrationCollectsDone >= CALIBRATION_COLLECT_SIZE;
  }


  public void rollerIntake() {
    m_roller.set(rollerIntakeSpeed.getValue());
  }

  public void rollerStop() {
    m_roller.stopMotor();
  }

  public void rollerOutgest() {
    m_roller.set(rolleroutgestSpeed.getValue());
  }

  public boolean isIntaking() {
    return m_roller.get() > 0.001;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake Roller Alive", m_roller.isAlive());

    if (!RobotContainer.isPublishingEnabled()) {
      return;
    }

    SmartDashboard.putNumber("Intake Arm Position", getArmPosition());
    SmartDashboard.putNumber("Intake Arm Velocity", getArmVelocity());
    SmartDashboard.putNumber("Intake Arm Current", m_arm.getSupplyCurrent());
    SmartDashboard.putBoolean("Intake Arm Deploy Calibrated", m_armDeployCalibrated);
    SmartDashboard.putBoolean("Intake Arm Stow Calibrated", m_armStowCalibrated);
    SmartDashboard.putBoolean("Intake Arm Deploy Limit", isDeployLimitTriggered());
    SmartDashboard.putBoolean("Intake Arm Stow Limit", isStowLimitTriggered());
    SmartDashboard.putNumber("Intake Arm Stowed Position", m_armStowed);
    SmartDashboard.putNumber("Intake Arm Deployed Position", m_armDeployed);

    SmartDashboard.putNumber("Intake Roller Current", m_arm.getSupplyCurrent());

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