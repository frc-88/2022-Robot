// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.CargoSource;
import frc.robot.util.CargoTarget;
import frc.robot.util.NumberCache;
import frc.robot.util.coprocessor.networktables.SwerveTable;
import frc.robot.util.ValueInterpolator;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

/**
 * Cargo is coming
 * Is it blue or is it red?
 * Mine scores, yours bounces
 **/

public class Shooter extends SubsystemBase implements CargoTarget {
  private TalonFX m_flywheel = new TalonFX(Constants.SHOOTER_FLYWHEEL_ID, "1");
  private TalonFX m_flywheelFollower = new TalonFX(Constants.SHOOTER_FLYWHEEL_FOLLOWER_ID, "1");
  private TalonFX m_hood = new TalonFX(Constants.HOOD_ID, "1");
  private Feeder m_feeder;
  private Sensors m_sensors;
  private Turret m_turret;
  private SwerveDrive m_drive;
  private SwerveTable m_ros_interface;
  private DoubleSupplier m_shotProbabilitySupplier;

  private static enum ActiveMode {
    DEACTIVATED,
    ACTIVE_PERMISSIVE,
    ACTIVE_RESTRICTIVE
  }

  private ActiveMode m_active = ActiveMode.DEACTIVATED;
  private Timer m_cargoWaitTimer = new Timer();
  private boolean m_cargoWaiting = false;
  private boolean m_wantedCargo = false;
  private Timer m_restrictiveCheckTimer = new Timer();
  private Timer m_rampDownTimer = new Timer();
  private static final double RAMP_DOWN_TIME = 1.5;
  private SlewRateLimiter m_flywheelLimiter = new SlewRateLimiter(5000);

  private double m_limelightDistance = 0;
  private double m_limelightAngle = 0;
  private double m_rosDistance = 0;
  private double m_rosAngle = 0;

  private static final double FLYWHEEL_RATIO = 40./24.;
  private static final double HOOD_RATIO = 17. * 218. / 12.;
  private static final double HOOD_DOWN = 13.;

  private final ValueInterpolator flywheelInterpolator = new ValueInterpolator(
    new ValueInterpolator.ValuePair(66.6739, 1400, 13),
    new ValueInterpolator.ValuePair(73.7607, 1450, 13),
    new ValueInterpolator.ValuePair(90.29, 1350, 19),
    new ValueInterpolator.ValuePair(108.77, 1375, 24),
    new ValueInterpolator.ValuePair(127.7, 1460, 24),
    new ValueInterpolator.ValuePair(156.5, 1575, 28.1),
    new ValueInterpolator.ValuePair(173, 1675, 31.2),
    new ValueInterpolator.ValuePair(193.5, 1735, 33.9),
    new ValueInterpolator.ValuePair(218.7, 1775, 33.6),
    new ValueInterpolator.ValuePair(260.47, 1825, 32.3),
    new ValueInterpolator.ValuePair(284.6, 1925, 36.2));

  // Preferences
  private PIDPreferenceConstants p_flywheelPID = new PIDPreferenceConstants("Shooter PID", 0.0, 0.0, 0.0, 0.047, 0.0,
      0.0, 0.0);
  private DoublePreferenceConstant p_flywheelIdle = new DoublePreferenceConstant("Shooter Idle Speed", 1150.0);
  private DoublePreferenceConstant p_shooterReady = new DoublePreferenceConstant("Shooter Pause (s)", 0.5);
  private DoublePreferenceConstant p_cargoInShooter = new DoublePreferenceConstant("Cargo In Shooter (s)", 0.2);
  private DoublePreferenceConstant p_shooterSpinLimit = new DoublePreferenceConstant("Shooter Spin Limit", 90.);
  private DoublePreferenceConstant p_shooterAccelerationLimit = new DoublePreferenceConstant("Shooter Acceleration Limit", 4.);
  private DoublePreferenceConstant p_shooterProbabilityLimit = new DoublePreferenceConstant("Shooter Probability Limit", 0.4);
  private DoublePreferenceConstant p_restrictiveCheckTime = new DoublePreferenceConstant("Shooter Restrictive Check Time", 0.2);

  private DoublePreferenceConstant p_hoodMaxVelocity = new DoublePreferenceConstant("Hood Max Velocity", 360);
  private DoublePreferenceConstant p_hoodMaxAcceleration = new DoublePreferenceConstant("Hood Max Acceleration", 1080);
  private PIDPreferenceConstants p_hoodPID = new PIDPreferenceConstants("Hood", 0, 0, 0, 0, 0, 0, 0);

  /** Creates a new Shooter. */
  public Shooter(Sensors sensors, SwerveDrive drive, Turret turret, Feeder feeder, SwerveTable ros_interface) {
    m_sensors = sensors;
    m_drive = drive;
    m_turret = turret;
    m_feeder = feeder;
    m_ros_interface = ros_interface;
    m_shotProbabilitySupplier = () -> 1.;

    m_restrictiveCheckTimer.reset();
    m_restrictiveCheckTimer.start();

    configureFlywheel();
    configureHood();

    p_flywheelPID.addChangeHandler((Double unused) -> configureFlywheel());
    p_hoodMaxVelocity.addChangeHandler((Double unused) -> configureHood());
    p_hoodMaxAcceleration.addChangeHandler((Double unused) -> configureHood());
    p_hoodPID.addChangeHandler((Double unused) -> configureHood());

    m_hood.setSelectedSensorPosition(convertHoodPositionToMotor(HOOD_DOWN));
  }

  private void configureFlywheel() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    config.slot0.kP = p_flywheelPID.getKP().getValue();
    config.slot0.kI = p_flywheelPID.getKI().getValue();
    config.slot0.kD = p_flywheelPID.getKD().getValue();
    config.slot0.kF = p_flywheelPID.getKF().getValue();
    config.slot0.integralZone = p_flywheelPID.getIZone().getValue();
    config.slot0.maxIntegralAccumulator = p_flywheelPID.getIMax().getValue();
    m_flywheel.configAllSettings(config);

    //Configures Follower
    m_flywheelFollower.configFactoryDefault();
    m_flywheelFollower.follow(m_flywheel);
    m_flywheelFollower.setInverted(TalonFXInvertType.OpposeMaster);
  }

  private void configureHood(){
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.neutralDeadband = 0;
    config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    config.motionCruiseVelocity = convertHoodVelocityToMotor(p_hoodMaxVelocity.getValue());
    config.motionAcceleration = convertHoodVelocityToMotor(p_hoodMaxAcceleration.getValue());
    config.slot0.kP = p_hoodPID.getKP().getValue();
    config.slot0.kI = p_hoodPID.getKI().getValue();
    config.slot0.kD = p_hoodPID.getKD().getValue();
    config.slot0.kF = p_hoodPID.getKF().getValue();
    config.slot0.integralZone = p_hoodPID.getIZone().getValue();
    config.slot0.maxIntegralAccumulator = p_hoodPID.getIMax().getValue();
    m_hood.configAllSettings(config);
    m_hood.setInverted(InvertType.InvertMotorOutput);
  }

  public void calibrateHood() {
    m_hood.setSelectedSensorPosition(convertHoodPositionToMotor(HOOD_DOWN));
  }


  public void setFlywheelSpeed(double speed) {
    m_flywheel.set(TalonFXControlMode.Velocity, m_flywheelLimiter.calculate(convertRPMsToMotorTicks(speed)));
  }

  public void setFlywheelRaw(double percentOutput) {
    m_flywheelLimiter.reset(0);
    m_flywheel.set(TalonFXControlMode.PercentOutput, percentOutput);
  }

  public void setFlywheelSpeedAuto(double target_dist) {
    if (m_feeder.hasBallInChamber()) {
      m_rampDownTimer.reset();
      m_rampDownTimer.start();
    }

    if (m_rampDownTimer.hasElapsed(RAMP_DOWN_TIME)) {
      setFlywheelRaw(0);
      setHoodMotionMagic(getHoodPosition());
    } else if (!m_turret.isTracking()) {
      setFlywheelSpeed(p_flywheelIdle.getValue());
      setHoodMotionMagic(HOOD_DOWN);
    } else {
      double[] values = calcSpeedFromDistance(target_dist);
      setFlywheelSpeed(values[0]);
      setHoodMotionMagic(values[1]);
    }
  }

  public void setFlywheelFenderShot() {
    setFlywheelSpeed(p_flywheelIdle.getValue());
    setHoodMotionMagic(HOOD_DOWN);
  }

  private void setHoodMotionMagic(double setpoint) {
    if (setpoint > 38) {
      setpoint = 38;
    } else if (setpoint < HOOD_DOWN) {
      setpoint = HOOD_DOWN;
    }
    m_hood.set(TalonFXControlMode.MotionMagic, convertHoodPositionToMotor(setpoint));
  }

  public void hoodDown() {
    setHoodMotionMagic(HOOD_DOWN);
  }

  private double[] calcSpeedFromDistance(double target_dist) {
    if (target_dist > 0.0) {
      return flywheelInterpolator.getInterpolatedValue(target_dist);
    } else {
      return new double[]{p_flywheelIdle.getValue(), HOOD_DOWN};
    }
  }

  public boolean onTarget() {
    return Math.abs(convertMotorTicksToRPM(m_flywheel.getClosedLoopError())) < p_flywheelPID.getTolerance().getValue();
  }

  public void activatePermissive() {
    m_active = ActiveMode.ACTIVE_PERMISSIVE;
  }

  public void activateRestrictive() {
    m_active = ActiveMode.ACTIVE_RESTRICTIVE;
  }

  public void deactivate() {
    m_active = ActiveMode.DEACTIVATED;
  }

  private double convertMotorTicksToRPM(double motorVelocity) {
    return motorVelocity * 600 / (FLYWHEEL_RATIO * 2048);
  }

  private double convertRPMsToMotorTicks(double flywheelVelocity) {
    return flywheelVelocity * FLYWHEEL_RATIO * 2048 / 600;
  }

  public boolean sourcesHaveCargo() {
    return m_feeder.hasBallInChamber() || m_feeder.hasBallInCentralizer();
  }

  public boolean cargoChambered() {
    return m_feeder.hasBallInChamber();
  }

  public boolean cargoCentralized() {
    return m_feeder.hasBallInCentralizer();
  }

  @Override
  public boolean wantsCargo() {
    // Permissive checks
    boolean flywheelOnTarget = onTarget();
    boolean turretOnTarget = m_turret.onTarget();
    boolean permissiveChecks = flywheelOnTarget && turretOnTarget;

    // Restrictive checks
    boolean turretTracking = m_turret.isTracking();
    boolean turretNotMoving = m_turret.notMoving();
    boolean driveNotSpinning = Math.abs(m_sensors.ahrs_navx.getRate()) <= p_shooterSpinLimit.getValue();
    boolean driveNotAccelerating = m_drive.getAccelerationEstimate() <= p_shooterAccelerationLimit.getValue();
    boolean highShotProbability = m_shotProbabilitySupplier.getAsDouble() >= p_shooterProbabilityLimit.getValue();
    boolean untimedChecks = permissiveChecks && turretTracking && turretNotMoving && driveNotSpinning && driveNotAccelerating && highShotProbability;
    boolean restrictiveChecks = false;
    if (m_wantedCargo && m_feeder.hasBallInChamber()) {
      restrictiveChecks = true;
    } else {
      if (m_wantedCargo) {
        m_restrictiveCheckTimer.reset();
      }
      if (untimedChecks && m_restrictiveCheckTimer.hasElapsed(p_restrictiveCheckTime.getValue())) {
        restrictiveChecks = true;
      } else if (!untimedChecks) {
        m_restrictiveCheckTimer.reset();
      }
    }
    

    boolean wantsCargo = (m_active == ActiveMode.ACTIVE_PERMISSIVE && permissiveChecks) || (m_active == ActiveMode.ACTIVE_RESTRICTIVE && restrictiveChecks);

    // if (m_active == ActiveMode.ACTIVE_PERMISSIVE && !wantsCargo) {
    //   String printString = "@Permissive Blocked: ";

    //   if (!flywheelOnTarget) {
    //     printString += "<Flywheel error is " + convertMotorTicksToRPM(m_flywheel.getClosedLoopError()) + "> ";
    //   }

    //   if (!turretOnTarget) {
    //     printString += "<Turret target is " + m_turret.getTarget() + " but facing is " + m_turret.getFacing() + "> ";
    //   }

    //   // System.out.println(printString);
    // } else if (m_active == ActiveMode.ACTIVE_RESTRICTIVE && !wantsCargo) {
    //   String printString = "@Restrictive Blocked: ";

    //   if (!flywheelOnTarget) {
    //     printString += "<Flywheel error is " + convertMotorTicksToRPM(m_flywheel.getClosedLoopError()) + "> ";
    //   }

    //   if (!turretTracking) {
    //     printString += "<Turret is not tracking> ";
    //   } else if (!turretOnTarget) {
    //     printString += "<Turret target is " + m_turret.getTarget() + " but facing is " + m_turret.getFacing() + "> ";
    //   }

    //   if (!driveNotSpinning) {
    //     printString += "<Yaw Rate is " + m_sensors.navx.getYawRate() + "> ";
    //   }

    //   if (!driveNotAccelerating) {
    //     printString += "<Acceleration estimate is " + m_drive.getAccelerationEstimate() + "> ";
    //   }

    //   if (!hoodNotMoving) {
    //     printString += "<Hood state is " + m_hood.getHoodState() + "> ";
    //   }

    //   if (!highShotProbability) {
    //     printString += "<Shot probability is " + m_shotProbabilitySupplier.getAsDouble() + "> ";
    //   }

    //   if (untimedChecks) {
    //     printString += "<Only " + m_restrictiveCheckTimer.get() + " has elapsed> ";
    //   }

    //   // System.out.println(printString);
    // } else 
    if (m_active != ActiveMode.DEACTIVATED && wantsCargo && !m_wantedCargo) {
      double flywheelVelocity = convertMotorTicksToRPM(m_flywheel.getSelectedSensorVelocity());
      m_ros_interface.signalShot(m_turret.getFacing(), m_rosDistance, flywheelVelocity);

      // System.out.println("@Shot: <Mode is " + m_active.toString() + "> " 
      //                  + "<Distance is " + m_limelightDistance + " from limelight and " + m_rosDistance + " from ROS, flywheel speed is " + flywheelVelocity + "> " 
      //                  + "<Angle is " + m_limelightAngle + " from limelight and " + m_rosAngle + " from ROS, turret angle is" + m_turret.getFacing() + "> " 
      //                  + "<Drive speed is " + m_drive.getStraightSpeed() + " with an acceleration estimate of " + m_drive.getAccelerationEstimate() + " and a yaw rate of " + m_sensors.navx.getYawRate() + "> " 
      //                  + "<Shot probability is " + m_ros_interface.getShooterProbability() + ">");
    }

    m_wantedCargo = wantsCargo;
    return wantsCargo;
  }

  public void registerLimelightTarget(double distance, double angle) {
    m_limelightDistance = distance;
    m_limelightAngle = angle;
  }
  
  public void registerROSTarget(double distance, double angle) {
    m_rosDistance = distance;
    m_rosAngle = angle;
  }

  private double convertMotorPositionToHood(double motorPosition) {
    return (motorPosition / (HOOD_RATIO * 2048.)) * 360.0;
  }

  private double convertMotorVelocityToHood(double motorVelocity) {
    return convertMotorPositionToHood(motorVelocity) * 10;
  }

  private double convertHoodPositionToMotor(double hoodPosition) {
    return (hoodPosition / 360.0) * HOOD_RATIO * 2048.;
  }

  private double convertHoodVelocityToMotor(double hoodVelocity) {
    return convertHoodPositionToMotor(hoodVelocity) / 10;
  }

  public double getHoodPosition() {
    if (NumberCache.hasValue("Hood Position")) {
      return NumberCache.getValue("Hood Position");
    }

    return NumberCache.pushValue("Hood Position", convertMotorPositionToHood(m_hood.getSelectedSensorPosition()));
  }

  @Override
  public void periodic() {

    if (!RobotContainer.isPublishingEnabled()) {
      return;
    }
    
    SmartDashboard.putNumber("Shooter Flywheel Velocity", convertMotorTicksToRPM(m_flywheel.getSelectedSensorVelocity())
        );
    SmartDashboard.putNumber("Shooter Flywheel Commanded Velocity",
        convertMotorTicksToRPM(m_flywheel.getClosedLoopTarget()));
    SmartDashboard.putBoolean("Shooter Flywheel On Target", onTarget());
    // SmartDashboard.putNumber("Flywheel Speed from Limelight", calcSpeedFromDistance(??));
    SmartDashboard.putBoolean("Shooter Wants Cargo", wantsCargo());
    
    SmartDashboard.putNumber("Hood Position", getHoodPosition());
  }
}
