/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.Queue;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.BatteryParamEstimator;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.sensors.Limelight;

/**
 * we gather data
 * about the world around us
 * to make good choices
 */

public class Sensors extends SubsystemBase {
  public final AHRS ahrs_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
  public final Limelight limelight = new Limelight();
  private final DigitalInput coastButton = new DigitalInput(Constants.SENSORS_COAST_BUTTON_ID);

  // private final ColorSensorV3 m_colorSensor = new ColorSensorV3(Port.kOnboard);
  private final PneumaticHub m_pneumaticHub = new PneumaticHub();
  private final PowerDistribution m_pdb = new PowerDistribution();
  private final Servo m_cameraTilter = new Servo(Constants.CAMERA_TILTER_SERVO_CHANNEL);
  private final BatteryParamEstimator m_batteryEstimator = new BatteryParamEstimator(500);

  private DoublePreferenceConstant p_colorRedThreshold = new DoublePreferenceConstant("Color Red Threshold", 0.0);
  private DoublePreferenceConstant p_colorBlueThreshold = new DoublePreferenceConstant("Color Blue Threshold", 0.0);

  private DoublePreferenceConstant p_cameraTiltLevelAngle = new DoublePreferenceConstant("Camera tilt level angle", Constants.CAMERA_TILT_LEVEL_ANGLE);
  private DoublePreferenceConstant p_cameraTiltDownCommand = new DoublePreferenceConstant("Camera tilt down command", Constants.CAMERA_TILT_DOWN_COMMAND);

  // First value is measurement time in minutes second is storage pressure in PSI
  private Queue<Pair<Double, Double>> m_storagePressureMeasurements = new LinkedList<Pair<Double, Double>>();

  private boolean checkedStoragePressure = false;
  private double currentStorageVoltage = 0;
  private boolean checkedWorkingPressure = false;
  private double currentWorkingVoltage = 0;

  private boolean m_compressorEnabled = true;

  /**
   * Creates a new Sensors subsystem
   */
  public Sensors() {
    CameraServer.startAutomaticCapture();
    setCameraTilterAngle(Constants.CAMERA_TILT_DOWN_ANGLE);

    SmartDashboard.putBoolean("Virtual Coast Button", false);
  }

  public boolean isCoastButtonPressed() {
    return !coastButton.get() || SmartDashboard.getBoolean("Virtual Coast Button", false);
  }

  public double getStoragePressure() {
    if (Robot.isReal()) {
      if (!checkedStoragePressure) {
        currentStorageVoltage = m_pneumaticHub.getAnalogVoltage(Constants.STORAGE_PRESSURE_SENSOR_CHANNEL);
        checkedStoragePressure = true;
      }
      return (currentStorageVoltage - 0.4132) / 0.0265;
    } else {
      return 120.;
    }
  }

  public double getWorkingPressure() {
    if (Robot.isReal()) {
      if (!checkedWorkingPressure) {
        currentWorkingVoltage = m_pneumaticHub.getAnalogVoltage(Constants.WORKING_PRESSURE_SENSOR_CHANNEL);
        checkedWorkingPressure = true;
      }
      return (currentWorkingVoltage - 0.3944) / 0.0396;
    } else {
      return 60.;
    }
  }

  public boolean isStoragePressureSensorConnected() {
    if (Robot.isSimulation()) {
      return true;
    }
    return currentStorageVoltage > Constants.PRESSURE_SENSOR_MIN_VOLTAGE
        && currentStorageVoltage < Constants.PRESSURE_SENSOR_MAX_VOLTAGE;
  }

  public boolean isWorkingPressureSensorConnected() {
    if (Robot.isSimulation()) {
      return true;
    }
    return currentWorkingVoltage > Constants.PRESSURE_SENSOR_MIN_VOLTAGE
        && currentWorkingVoltage < Constants.PRESSURE_SENSOR_MAX_VOLTAGE;
  }

  public void setCameraTilterAngle(Rotation2d angle) {
    setCameraTilterAngle(angle.getDegrees());
  }

  public void setCameraTilterAngle(double angleDegrees) {
    SmartDashboard.putNumber("Camera tilter degrees", angleDegrees);
    SmartDashboard.putNumber("Camera tilter command degrees", convertAngleToServoCommand(angleDegrees));
    m_cameraTilter.setAngle(convertAngleToServoCommand(angleDegrees));
  }

  public Rotation2d getCameraTilterAngle() {
    double angleDegrees = convertServoCommandToAngle(m_cameraTilter.getAngle());

    SmartDashboard.putNumber("Camera tilter read degrees", angleDegrees);
    return Rotation2d.fromDegrees(angleDegrees);
  }

  public double convertServoCommandToAngle(double servoCommand) {
    return -(servoCommand - p_cameraTiltDownCommand.getValue()) - p_cameraTiltLevelAngle.getValue();
  }

  public double convertAngleToServoCommand(double angleDegrees) {
    return -(angleDegrees + p_cameraTiltLevelAngle.getValue()) + p_cameraTiltDownCommand.getValue();
  }

  public boolean isCargoOurs() {
    // Color detectedColor = m_colorSensor.getColor();
    boolean foundOurs = true;

    // if (DriverStation.getAlliance() == Alliance.Red) {
    //   if (detectedColor.red > p_colorRedThreshold.getValue() &&
    //       detectedColor.blue < p_colorBlueThreshold.getValue()) {
    //         foundOurs = true;
    //   }
    // } else {
    //   if (detectedColor.red < p_colorRedThreshold.getValue() &&
    //       detectedColor.blue > p_colorBlueThreshold.getValue()) {
    //         foundOurs = true;
    //   }
    // }

    return foundOurs;
  }

  public void firstPeriodic() {

    if (DriverStation.isEnabled()) {
      SmartDashboard.putBoolean("Virtual Coast Button", false);
    }
    SmartDashboard.putBoolean("Coast Button", isCoastButtonPressed());
    limelight.periodic();

    if (m_compressorEnabled && DriverStation.isAutonomous()) {
      m_compressorEnabled = false;
      m_pneumaticHub.disableCompressor();
    } else if (!m_compressorEnabled && DriverStation.isTeleop()) {
      m_compressorEnabled = true;
      m_pneumaticHub.enableCompressorDigital();
    }

    if (!RobotContainer.isPublishingEnabled()) {
      return;
    }

    checkedStoragePressure = false;
    checkedWorkingPressure = false;
    getStoragePressure();
    getWorkingPressure();

    // NavX data
    SmartDashboard.putNumber("NavX Yaw", ahrs_navx.getYaw());
    SmartDashboard.putNumber("NavX Yaw Rate", ahrs_navx.getRate());
    SmartDashboard.putNumber("NavX Pitch", ahrs_navx.getPitch());
    SmartDashboard.putNumber("NavX Roll", ahrs_navx.getRoll());

    // Limelight calculations
    SmartDashboard.putNumber("Limelight Distance", limelight.getTargetDistance());
    SmartDashboard.putNumber("Limelight Angle", limelight.getCalibrationAngle());
    SmartDashboard.putNumber("Limelight Turret Offset", limelight.getTurretOffset());
    SmartDashboard.putBoolean("Limelight Has Target?", limelight.hasTarget());
    SmartDashboard.putBoolean("Limelight On Target?", limelight.onTarget());

    // Battery estimator
    m_batteryEstimator.updateEstimate(m_pdb.getVoltage(), m_pdb.getTotalCurrent());
    SmartDashboard.putNumber("Battery ESR", m_batteryEstimator.getEstESR());
    SmartDashboard.putNumber("Battery VOC", m_batteryEstimator.getEstVoc());
    SmartDashboard.putNumber("Battery Energy Used", m_pdb.getTotalEnergy());

    // Color Sensor data
    // Color detectedColor = m_colorSensor.getColor();
    // double IR = m_colorSensor.getIR();

    // SmartDashboard.putNumber("Red", detectedColor.red);
    // SmartDashboard.putNumber("Green", detectedColor.green);
    // SmartDashboard.putNumber("Blue", detectedColor.blue);
    // SmartDashboard.putNumber("IR", IR);
    // SmartDashboard.putBoolean("Our Cargo", isCargoOurs());

    // Pressure tracking
    // double storagePressure = this.getStoragePressure();
    // double workingPressure = this.getWorkingPressure();
    // double pressureDifference;
    // double leakRatePSI;
    // double leakRatePercentage;
    // if (DriverStation.isEnabled()) {
    //   // Only track leaks while disabled, clear history once enabled
    //   pressureDifference = 0;
    //   leakRatePSI = 0;
    //   leakRatePercentage = 0;
    //   m_storagePressureMeasurements.clear();
    // } else {
    //   double measurementTime = RobotController.getFPGATime() / 60E6; // Time in minutes
    //   if (m_storagePressureMeasurements.size() > 0) {
    //     pressureDifference = m_storagePressureMeasurements.peek().getSecond() - storagePressure;
    //     while (pressureDifference > Constants.PRESSURE_DIFFERENCE_TARGET && m_storagePressureMeasurements.size() > 1) {
    //       // Measure the time it took for the last 5 PSI to leak (or since last enabled if
    //       // 5 PSI hasn't been leaked yet)
    //       m_storagePressureMeasurements.poll();
    //       pressureDifference = m_storagePressureMeasurements.peek().getSecond() - storagePressure;
    //     }

    //     double timeDifference = measurementTime - m_storagePressureMeasurements.peek().getFirst();
    //     if (timeDifference > 0.) {
    //       // The leak rate in PSI/minute
    //       leakRatePSI = pressureDifference / timeDifference;
    //       double averagePressure = (m_storagePressureMeasurements.peek().getSecond() + storagePressure) / 2.;
    //       if (averagePressure > 0) {
    //         // The leak rate as a percentage of total pressure lost each minute
    //         leakRatePercentage = leakRatePSI
    //             / ((m_storagePressureMeasurements.peek().getSecond() + storagePressure) / 2.);
    //       } else {
    //         // Don't divide by 0
    //         leakRatePercentage = 0;
    //       }
    //     } else {
    //       // Don't divide by 0
    //       leakRatePSI = 0;
    //       leakRatePercentage = 0;
    //     }
    //   } else {
    //     // Can't measure for leaks with no measurement history
    //     pressureDifference = 0;
    //     leakRatePSI = 0;
    //     leakRatePercentage = 0;
    //   }
    //   m_storagePressureMeasurements.add(new Pair<Double, Double>(measurementTime, storagePressure));
    // }
    // SmartDashboard.putNumber("Storage Pressure", storagePressure);
    // SmartDashboard.putNumber("Working Pressure", workingPressure);
    // SmartDashboard.putNumber("Leak Pressure Difference PSI", pressureDifference);
    // SmartDashboard.putNumber("Leak Rate PSI per minute", leakRatePSI);
    // SmartDashboard.putNumber("Leak Rate Percentage per minute", leakRatePercentage);
    // SmartDashboard.putBoolean("Working Pressure Warning", workingPressure < Constants.WORKING_PRESSURE_WARNING);
    // SmartDashboard.putBoolean("Leak Warning", leakRatePercentage > Constants.LEAK_WARNING);
    // SmartDashboard.putBoolean("Storage Pressure Sensor Disconnected", !this.isStoragePressureSensorConnected());
    // SmartDashboard.putBoolean("Working Pressure Sensor Disconnected", !this.isWorkingPressureSensorConnected());
  }
}
