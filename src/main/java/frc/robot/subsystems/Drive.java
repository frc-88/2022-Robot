/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Objects;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.SyncPIDController;
import frc.robot.util.drive.DriveConfiguration;
import frc.robot.util.drive.DriveUtils;
import frc.robot.util.drive.Shifter;
import frc.robot.util.drive.TJDriveModule;
import frc.robot.util.drive.Shifter.Gear;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;
import frc.robot.util.transmission.CTREMagEncoder;
import frc.robot.util.transmission.Falcon500;
import frc.robot.util.transmission.ShiftingTransmission;
import frc.robot.util.coprocessortable.ChassisInterface;
import frc.robot.util.coprocessortable.VelocityCommand;

public class Drive extends SubsystemBase implements ChassisInterface {
  
  private final Sensors m_sensors;

  private final TJDriveModule m_leftDrive, m_rightDrive;
  private final WPI_CANCoder m_leftEncoder, m_rightEncoder;

  private final ShiftingTransmission m_leftTransmission, m_rightTransmission;
  private final SyncPIDController m_leftVelPID, m_rightVelPID;
  private final DriveConfiguration m_driveConfiguration;
  private final Shifter m_leftShifter, m_rightShifter;

  private double m_leftCommandedSpeed = 0;
  private double m_rightCommandedSpeed = 0;
  private double m_maxSpeed = Constants.MAX_SPEED_HIGH;
  private double m_maxAngVelHighGear = 6.0;  // in radians per second. arc / radius = angle
  // private double m_maxAngVelHighGear = 2.0 * Constants.MAX_SPEED_HIGH / Constants.WHEEL_BASE_WIDTH;  // in radians per second. arc / radius = angle
  private double m_maxAngVelLowGear = 2.0 * Constants.MAX_SPEED_LOW / Constants.WHEEL_BASE_WIDTH;  // in radians per second. arc / radius = angle

  private DifferentialDriveKinematics m_kinematics;
  private DifferentialDriveOdometry m_odometry;
  private Pose2d m_pose;
  private Pose2d m_traj_reset_pose;
  private Pose2d m_traj_offset;

  private PIDPreferenceConstants velPIDConstants;
  private DoublePreferenceConstant downshiftSpeed;
  private DoublePreferenceConstant upshiftSpeed;
  private DoublePreferenceConstant commandDownshiftSpeed;
  private DoublePreferenceConstant commandDownshiftCommandValue;
  private DoublePreferenceConstant lowStaticFriction;
  private DoublePreferenceConstant highStaticFriction;
  private DoublePreferenceConstant leftLowEfficiency;
  private DoublePreferenceConstant rightLowEfficiency;
  private DoublePreferenceConstant leftHighEfficiency;
  private DoublePreferenceConstant rightHighEfficiency;
  private DoublePreferenceConstant maxCurrent;
  private DoublePreferenceConstant universalCurrentLimit;

  // Constants for negative inertia
  private static final double LARGE_TURN_RATE_THRESHOLD = 0.65;
  private static final double INCREASE_TURN_SCALAR = 2;
  private static final double SMALL_DECREASE_TURN_SCALAR = 2.0;
  private static final double LARGE_DECREASE_TURN_SCALAR = 2.5;
  private double m_prevTurn = 0; // The last turn value
  private double m_negInertialAccumulator = 0; // Accumulates our current inertia value

  // Simulation
  private DifferentialDrivetrainSim m_driveSim;
  private SimDouble m_gyroSim;
  private Field2d m_field;

  // Acceleration limiting
  private DoublePreferenceConstant accelLimit;

  // Locking
  private boolean m_locked = false;

  public Drive(Sensors sensors) {
    m_sensors = sensors;

    m_driveConfiguration = new DriveConfiguration();

    velPIDConstants = new PIDPreferenceConstants("Drive Vel", 1.0, 0.02, 0, 0, 2, 2, 0);
    downshiftSpeed = new DoublePreferenceConstant("Downshift Speed", 4.5);
    upshiftSpeed = new DoublePreferenceConstant("UpshiftSpeed", 6);
    commandDownshiftSpeed = new DoublePreferenceConstant("Command Downshift Speed", 5);
    commandDownshiftCommandValue = new DoublePreferenceConstant("Command Downshift Command Value", 0.1);
    lowStaticFriction = new DoublePreferenceConstant("Drive Low Static Friction", Constants.DRIVE_LOW_STATIC_FRICTION_VOLTAGE);
    highStaticFriction = new DoublePreferenceConstant("Drive High Static Friction", Constants.DRIVE_HIGH_STATIC_FRICTION_VOLTAGE);
    leftLowEfficiency = new DoublePreferenceConstant("Drive Left Low Efficiency", Constants.DRIVE_LEFT_LOW_EFFICIENCY);
    rightLowEfficiency = new DoublePreferenceConstant("Drive Right Low Efficiency", Constants.DRIVE_RIGHT_LOW_EFFICIENCY);
    leftHighEfficiency = new DoublePreferenceConstant("Drive Left High Efficiency", Constants.DRIVE_LEFT_HIGH_EFFICIENCY);
    rightHighEfficiency = new DoublePreferenceConstant("Drive Right High Efficiency", Constants.DRIVE_RIGHT_HIGH_EFFICIENCY);
    maxCurrent = new DoublePreferenceConstant("Drive Max Current", Constants.DRIVE_CURRENT_LIMIT);
    universalCurrentLimit = new DoublePreferenceConstant("Universal Current Limit", 600);
    accelLimit = new DoublePreferenceConstant("Drive Accel Limit", 1);

    m_leftTransmission = new ShiftingTransmission(new Falcon500(), Constants.NUM_DRIVE_MOTORS_PER_SIDE,
        new CTREMagEncoder(), Constants.LOW_DRIVE_RATIO, Constants.HIGH_DRIVE_RATIO, Constants.DRIVE_SENSOR_RATIO,
        lowStaticFriction.getValue(), highStaticFriction.getValue(),
        leftLowEfficiency.getValue(), leftHighEfficiency.getValue());
    m_rightTransmission = new ShiftingTransmission(new Falcon500(), Constants.NUM_DRIVE_MOTORS_PER_SIDE,
        new CTREMagEncoder(), Constants.LOW_DRIVE_RATIO, Constants.HIGH_DRIVE_RATIO, Constants.DRIVE_SENSOR_RATIO,
        lowStaticFriction.getValue(), highStaticFriction.getValue(),
        rightLowEfficiency.getValue(), rightHighEfficiency.getValue());

    m_leftVelPID = new SyncPIDController(velPIDConstants);
    m_rightVelPID = new SyncPIDController(velPIDConstants);
    m_leftTransmission.setVelocityPID(m_leftVelPID);
    m_rightTransmission.setVelocityPID(m_rightVelPID);

    m_leftEncoder = new WPI_CANCoder(Constants.LEFT_DRIVE_ENCODER_ID, "1");
    m_rightEncoder = new WPI_CANCoder(Constants.RIGHT_DRIVE_ENCODER_ID, "1");

    m_leftEncoder.configFactoryDefault();
    m_rightEncoder.configFactoryDefault();
 
    m_leftDrive = new TJDriveModule(m_driveConfiguration.left, m_leftTransmission);
    m_rightDrive = new TJDriveModule(m_driveConfiguration.right, m_rightTransmission);

    m_leftDrive.configRemoteFeedbackFilter(m_leftEncoder, 0);
    m_rightDrive.configRemoteFeedbackFilter(m_rightEncoder, 0);

    m_leftShifter = new Shifter(Constants.LEFT_SHIFTER_CONSTANTS, m_leftDrive);
    m_rightShifter = new Shifter(Constants.RIGHT_SHIFTER_CONSTANTS, m_rightDrive);

    m_driveSim = new DifferentialDrivetrainSim(
      DCMotor.getFalcon500(2),
      1. / Constants.LOW_GEAR_RATIO,
      6., // Moment of Inertia in kg m^2, guessed
      55., // 120lb robot, in kg
      Units.inchesToMeters(Constants.WHEEL_DIAMETER / 2.),
      Units.feetToMeters(Constants.WHEEL_BASE_WIDTH),
      VecBuilder.fill(0.001, 0.001, 0.001, 0.01, 0.01, 0.005, 0.005) // measurement noise, stolen from wpilib docs
    );

    if (Robot.isSimulation()) {
      m_gyroSim = new SimDouble(SimDeviceDataJNI.getSimValueHandle(SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));
    }

    m_field = new Field2d();
    SmartDashboard.putData(m_field);

    shiftToLow();

    SmartDashboard.putBoolean("Zero Drive", false);

    // Creating my kinematics object
    m_kinematics = new DifferentialDriveKinematics(Units.feetToMeters(Constants.WHEEL_BASE_WIDTH));

    // Creating my odometry object
    // our starting pose is 1 meters along the long end of the field and in the
    // center of the field along the short end, facing forward.
    m_pose = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0), new Rotation2d());
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-m_sensors.navx.getYaw()), m_pose);
  
    resetTrajectoryPose(new Pose2d());
  }

  public void basicDrive(double leftSpeed, double rightSpeed) {
    m_leftDrive.set(ControlMode.PercentOutput, leftSpeed);
    m_rightDrive.set(ControlMode.PercentOutput, rightSpeed);
  }

  /**
   * Commands the drivetrain to the given velocities (in fps) while proactively
   * limiting current draw.
   */
  public void basicDriveLimited(double leftVelocity, double rightVelocity) {
    if (m_locked) {
      leftVelocity = getLeftSpeed();
      rightVelocity = getRightSpeed();
    }

    double leftExpectedCurrent = m_leftDrive.getExpectedCurrentDraw(leftVelocity);
    double rightExpectedCurrent = m_rightDrive.getExpectedCurrentDraw(rightVelocity);
    double totalExpectedCurrent = leftExpectedCurrent + rightExpectedCurrent;
    double leftCurrentLimit;
    double rightCurrentLimit;
    double leftUniversalCurrentLimit;
    double rightUniversalCurrentLimit;
    if (totalExpectedCurrent == 0) {
      leftCurrentLimit =  maxCurrent.getValue() / 2.;
      rightCurrentLimit = maxCurrent.getValue() / 2.;
      leftUniversalCurrentLimit = universalCurrentLimit.getValue() / 2.;
      rightUniversalCurrentLimit = universalCurrentLimit.getValue() / 2.;
    } else {
      leftCurrentLimit = maxCurrent.getValue() * leftExpectedCurrent / totalExpectedCurrent;
      rightCurrentLimit = maxCurrent.getValue() * rightExpectedCurrent / totalExpectedCurrent;
      leftUniversalCurrentLimit = universalCurrentLimit.getValue() * leftExpectedCurrent / totalExpectedCurrent;
      rightUniversalCurrentLimit = universalCurrentLimit.getValue() * rightExpectedCurrent / totalExpectedCurrent;
    }

    m_leftDrive.setVelocityCurrentLimited(leftVelocity, leftCurrentLimit, leftUniversalCurrentLimit);
    m_rightDrive.setVelocityCurrentLimited(rightVelocity, rightCurrentLimit, rightUniversalCurrentLimit);
    
    m_leftCommandedSpeed = leftVelocity;
    m_rightCommandedSpeed = rightVelocity;
  }

  /**
   * Arcade drive function for teleop control.
   * 
   * Parameters:
   * 
   * @param speed    The forwards/backwards speed on a scale from -1 to 1
   * @param turnRate The rate to turn at on a scale from -1 (counterclockwise) to
   *                 1 (clockwise)
   */
  public void arcadeDrive(double speed, double turn) {
    // Apply negative intertia
    turn = negativeInertia(speed, turn);

    // Convert to feet per second
    speed *= m_maxSpeed;
    turn *= m_maxSpeed;

    speed = limitAcceleration(speed);
    
    // Calculate left and right speed
    double leftSpeed = (speed + turn);
    double rightSpeed = (speed - turn);

    // Apply values
    basicDriveLimited(leftSpeed, rightSpeed);
  }

  public void updateCurrentGear() {
    Gear leftGear = getLeftGear();
    Gear rightGear = getRightGear();
    if (leftGear == Gear.LOW) {
      m_leftTransmission.shiftToLow();
    } else if (leftGear == Gear.HIGH) {
      m_leftTransmission.shiftToHigh();
    }
    if (rightGear == Gear.LOW) {
      m_rightTransmission.shiftToLow();
    } else if (rightGear == Gear.HIGH) {
      m_rightTransmission.shiftToHigh();
    }
  }

  public double limitAcceleration(double speed) {
    double currentSpeed = getStraightSpeed();
    if (speed - currentSpeed > 0) {
        double vel = currentSpeed + accelLimit.getValue();
      if (speed < vel) {
        return speed;
      } else {
        return vel;
      }
    } else {
      double vel = getStraightSpeed() - accelLimit.getValue();;
      if (speed > vel) {
        return speed;
      } else {
        return vel;
      }
    }
  }

  public void autoshift(double commandedValue) {
    autoshiftSide(commandedValue, getLeftGear(), m_leftShifter, getLeftSpeed());
    autoshiftSide(commandedValue, getRightGear(), m_rightShifter, getRightSpeed());
  }

  public void autoshiftSide(double commandValue, Gear currentGear, Shifter shifter, double currentSpeed) {
    if (currentGear == Gear.HIGH && Math.abs(currentSpeed) <= downshiftSpeed.getValue()) {
      shifter.shiftToLow();
    } else if (currentGear == Gear.LOW && Math.abs(currentSpeed) >= upshiftSpeed.getValue()) {
      shifter.shiftToHigh();
    } else if (currentGear == Gear.HIGH && Math.abs(currentSpeed) <= commandDownshiftSpeed.getValue()
        && (Math.signum(commandValue) != Math.signum(currentSpeed)
            || Math.abs(commandValue) <= commandDownshiftCommandValue.getValue())) {
      shifter.shiftToLow();
    }
  }

  public void shiftToLow() {
    m_leftShifter.shiftToLow();
    m_rightShifter.shiftToLow();
  }

  public void shiftToHigh() {
    m_leftShifter.shiftToHigh();
    m_rightShifter.shiftToHigh();
  }

  public Gear getLeftGear() {
    return m_leftShifter.getGear();
  }

  public Gear getRightGear() {
    return m_rightShifter.getGear();
  }

  public void resetEncoderPositions() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double getLeftPosition() {
    return m_leftDrive.getScaledSensorPosition();
  }

  public double getRightPosition() {
    return m_rightDrive.getScaledSensorPosition();
  }

  public double getLeftSpeed() {
    return m_leftDrive.getScaledSensorVelocity();
  }

  public double getRightSpeed() {
    return m_rightDrive.getScaledSensorVelocity();
  }

  public double getStraightSpeed() {
    return (getLeftSpeed() + getRightSpeed()) / 2;
  }

  public void setBrakeMode() {
    m_leftDrive.brakeAll();
    m_rightDrive.brakeAll();
  }

  public void setCoastMode() {
    m_leftDrive.coastAll();
    m_rightDrive.coastAll();
  }

  public void lockDrive() {
    m_locked = true;
  }

  public void unlockDrive() {
    m_locked = false;
  }

  public void setMaxSpeed(double maxSpeed) {
    m_maxSpeed = maxSpeed;
  }

  public Pose2d getCurrentPose() {
    return m_pose;
  }

  public ChassisSpeeds getCurrentChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(Units.feetToMeters(getLeftSpeed()), Units.feetToMeters(getRightSpeed())));
  }

  public DifferentialDriveWheelSpeeds wheelSpeedsFromChassisSpeeds(ChassisSpeeds speeds) {
    return m_kinematics.toWheelSpeeds(speeds);
  }

  // Negative inertia! The idea is that the robot has some inertia
  // which theoretically is based on previously commanded values. Returns an
  // updated turn value
  private double negativeInertia(double throttle, double turn) {

      // How much we are currently trying to change the turn value
      double turnDiff = turn - m_prevTurn;
      m_prevTurn = turn;

      // Determine which scaling constant to use based on how we are moving
      double negInertiaScalar;
      if (turn * turnDiff > 0) {
          // We are trying to increase our turning rate
          negInertiaScalar = INCREASE_TURN_SCALAR;
      } else {
          if (Math.abs(turn) < LARGE_TURN_RATE_THRESHOLD) {
              // We are trying to reduce our turning rate to something
              // relatively close to 0
              negInertiaScalar = SMALL_DECREASE_TURN_SCALAR;
          } else {
              // We are trying to reduce our turning rate, but still want to
              // be turning fairly fast
              negInertiaScalar = LARGE_DECREASE_TURN_SCALAR;
          }
      }

      // Apply the scalar, and add it to the accumulator
      double negInertiaPower = turnDiff * negInertiaScalar;
      m_negInertialAccumulator += negInertiaPower;

      // Add the current negative inertia value to the turn
      double updatedTurn = turn + m_negInertialAccumulator;

      // Reduce our current inertia
      if (m_negInertialAccumulator > 1) {
          m_negInertialAccumulator -= 1;
      } else if (m_negInertialAccumulator < -1) {
          m_negInertialAccumulator += 1;
      } else {
          m_negInertialAccumulator = 0;
      }

      return updatedTurn;
  }

  public void zeroDrive() {
    m_sensors.navx.zeroYaw();
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
    m_leftDrive.setSelectedSensorPosition(0);
    m_rightDrive.setSelectedSensorPosition(0);
  }

  public void resetOdometry() {
    resetOdometry(new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0), new Rotation2d()), new Rotation2d());
  }

  public void resetOdometry(Pose2d startPose, Rotation2d startGyro) {
    m_odometry.resetPosition(startPose, startGyro);
  }

  public void resetTrajectoryPose(Pose2d startPose) {
    m_traj_reset_pose = m_odometry.getPoseMeters();
    m_traj_offset = startPose;
  }

  public void updateOdometry() {
    Pose2d odom_pose = m_odometry.update(Rotation2d.fromDegrees(-m_sensors.navx.getYaw()), Units.feetToMeters(getLeftPosition()), Units.feetToMeters(getRightPosition()));
    if (m_traj_reset_pose == null || m_traj_offset == null) {
      m_pose = odom_pose;
    } else {
      Pose2d reset_relative_pose = odom_pose.relativeTo(m_traj_reset_pose);
      m_pose = DriveUtils.relativeToReverse(reset_relative_pose, m_traj_offset);
    }
  }

  public Field2d getField() {
    return m_field;
  }

  @Override
  public void periodic() {
    if (SmartDashboard.getBoolean("Zero Drive", false)) {
      zeroDrive();
      resetOdometry();
      SmartDashboard.putBoolean("Zero Drive", false);
    }
    
    updateOdometry();
    m_field.setRobotPose(getCurrentPose());

    SmartDashboard.putNumber("L Drive Current", m_leftDrive.getTotalCurrent());
    SmartDashboard.putNumber("R Drive Current", m_rightDrive.getTotalCurrent());
    SmartDashboard.putNumber("L Drive Speed", m_leftDrive.getScaledSensorVelocity());
    SmartDashboard.putNumber("R Drive Speed", m_rightDrive.getScaledSensorVelocity());
    SmartDashboard.putNumber("L Drive Position", m_leftDrive.getScaledSensorPosition());
    SmartDashboard.putNumber("R Drive Position", m_rightDrive.getScaledSensorPosition());
    SmartDashboard.putNumber("L Drive Command Speed", m_leftCommandedSpeed);
    SmartDashboard.putNumber("R Drive Command Speed", m_rightCommandedSpeed);
    SmartDashboard.putNumber("L Drive Voltage", m_leftDrive.getMotorOutputVoltage());
    SmartDashboard.putNumber("R Drive Voltage", m_rightDrive.getMotorOutputVoltage());
    SmartDashboard.putString("L Drive Gear", getLeftGear().toString());
    SmartDashboard.putString("R Drive Gear", getRightGear().toString());
    SmartDashboard.putNumber("Straight Speed", this.getStraightSpeed());
    SmartDashboard.putNumber("Max Drive Speed", m_maxSpeed);

    SmartDashboard.putNumber("Pose X", Units.metersToFeet(m_pose.getX()));
    SmartDashboard.putNumber("Pose Y", Units.metersToFeet(m_pose.getY()));
    SmartDashboard.putNumber("Pose Rotation", m_pose.getRotation().getDegrees());
    ChassisSpeeds speeds = getCurrentChassisSpeeds();
    SmartDashboard.putNumber("Linear Velocity X", Units.metersToFeet(speeds.vxMetersPerSecond));
    SmartDashboard.putNumber("Linear Velocity Y", Units.metersToFeet(speeds.vyMetersPerSecond));
    SmartDashboard.putNumber("Angular Velocity", Units.metersToFeet(speeds.omegaRadiansPerSecond));

    if (DriverStation.isEnabled()) {
      this.setBrakeMode();
    } else {
      this.setCoastMode();
    }

    if (m_leftDrive.hasResetOccurred()) {
      m_leftDrive.setStatusFrames();
    }
    if (m_rightDrive.hasResetOccurred()) {
      m_rightDrive.setStatusFrames();
    }
  }

  // ROS tunnel interfaces
  @Override
  public void drive(VelocityCommand command) {
    drive(command.vx, command.vy, command.vt);
  }

  @Override
  public void drive(double vx, double vy, double angularVelocity) {
    // vx and vy in meters per second
    // angularVelocity in radians per second

    double vx_fps = vx * Constants.METERS_TO_FEET;
    double turn_fps = angularVelocity * Constants.WHEEL_BASE_WIDTH / 2.0;
    autoshift(0);  // 0 for aggresive shifting
    updateCurrentGear();
    basicDriveLimited(vx_fps - turn_fps, vx_fps + turn_fps);
  }

  @Override
  public void stop() {
    basicDrive(0.0, 0.0);
  }

  @Override
  public void resetPosition(Pose2d pose) {
    zeroDrive();
    resetOdometry(pose, pose.getRotation());
  }

  @Override
  public Pose2d getOdometryPose() {
    return m_odometry.getPoseMeters();
  }

  @Override
  public ChassisSpeeds getChassisVelocity() {
    return getCurrentChassisSpeeds();
  }

  @Override
  public void simulationPeriodic() {
    if (m_leftTransmission.isInHighGear()) {
      m_driveSim.setCurrentGearing(1. / Constants.HIGH_GEAR_RATIO);
    } else {
      m_driveSim.setCurrentGearing(1. / Constants.LOW_GEAR_RATIO);
    }

    int leftInversion = m_leftDrive.getInverted() ? -1 : 1;
    int rightInversion = m_rightDrive.getInverted() ? -1 : 1;

    m_driveSim.setInputs(
      m_leftDrive.getSimCollection().getMotorOutputLeadVoltage() * leftInversion, 
      m_rightDrive.getSimCollection().getMotorOutputLeadVoltage() * rightInversion
    );

    m_driveSim.update(0.02);

    m_leftEncoder.getSimCollection().setRawPosition((int)m_leftTransmission.convertOutputPositionToSensor(Units.metersToFeet(m_driveSim.getLeftPositionMeters()) * leftInversion));
    m_rightEncoder.getSimCollection().setRawPosition((int)m_rightTransmission.convertOutputPositionToSensor(Units.metersToFeet(m_driveSim.getRightPositionMeters()) * rightInversion));
    m_leftEncoder.getSimCollection().setVelocity(Math.max(-32767, Math.min(32767, (int)m_leftTransmission.convertOutputVelocityToSensor(Units.metersToFeet(m_driveSim.getLeftVelocityMetersPerSecond()) * leftInversion))));
    m_rightEncoder.getSimCollection().setVelocity(Math.max(-32767, Math.min(32767, (int)m_rightTransmission.convertOutputVelocityToSensor(Units.metersToFeet(m_driveSim.getRightVelocityMetersPerSecond()) * rightInversion))));

    m_gyroSim.set(m_driveSim.getHeading().getDegrees());
  }
}
