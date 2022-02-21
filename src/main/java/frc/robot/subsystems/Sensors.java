/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.Queue;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.sensors.Limelight;
import frc.robot.util.sensors.NavX;

/**
 * we gather data
 * about the world around us
 * to make good choices
 */

public class Sensors extends SubsystemBase {
  public final NavX navx = new NavX();
  public final Limelight limelight = new Limelight();
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(Constants.I2C_ONBOARD);
  private final PneumaticHub m_pneumaticHub = new PneumaticHub();

  // First value is measurement time in minutes second is storage pressure in PSI
  private Queue<Pair<Double, Double>> m_storagePressureMeasurements = new LinkedList<Pair<Double, Double>>();

  private boolean checkedStoragePressure = false;
  private double currentStorageVoltage = 0;
  private boolean checkedWorkingPressure = false;
  private double currentWorkingVoltage = 0;

  /**
   * Creates a new Sensors subsystem
   */
  public Sensors() {
    CameraServer.startAutomaticCapture();
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
    return currentStorageVoltage > Constants.PRESSURE_SENSOR_MIN_VOLTAGE && currentStorageVoltage < Constants.PRESSURE_SENSOR_MAX_VOLTAGE;
  }

  public boolean isWorkingPressureSensorConnected() {
    if (Robot.isSimulation()) {
      return true;
    }
    return currentWorkingVoltage > Constants.PRESSURE_SENSOR_MIN_VOLTAGE && currentWorkingVoltage < Constants.PRESSURE_SENSOR_MAX_VOLTAGE;
  }

  @Override
  public void periodic() {
    checkedStoragePressure = false;
    checkedWorkingPressure = false;
    getStoragePressure();
    getWorkingPressure();

    // NavX data
    SmartDashboard.putNumber("NavX Yaw", navx.getYaw());
    SmartDashboard.putNumber("NavX Yaw Rate", navx.getYawRate());
    SmartDashboard.putNumber("NavX Pitch", navx.getPitch());
    SmartDashboard.putNumber("NavX Roll", navx.getRoll());

    // Limelight calculations
    SmartDashboard.putNumber("Limelight Distance", limelight.calcDistanceToTarget());
    SmartDashboard.putNumber("Limelight Angle", limelight.calcLimelightAngle());
    SmartDashboard.putBoolean("Limelight Hood Up?", limelight.isHoodUp());

    // Color Sensor data
    Color detectedColor = m_colorSensor.getColor();
    double IR = m_colorSensor.getIR();

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);

    // Pressure tracking
    double storagePressure = this.getStoragePressure();
    double workingPressure = this.getWorkingPressure();
    double pressureDifference;
    double leakRatePSI;
    double leakRatePercentage;
    if (DriverStation.isEnabled()) {
      // Only track leaks while disabled, clear history once enabled
      pressureDifference = 0;
      leakRatePSI = 0;
      leakRatePercentage = 0;
      m_storagePressureMeasurements.clear();
    } else {
      double measurementTime = RobotController.getFPGATime() / 60E6; // Time in minutes
      if (m_storagePressureMeasurements.size() > 0) {
        pressureDifference = m_storagePressureMeasurements.peek().getSecond() - storagePressure;
        while (pressureDifference > Constants.PRESSURE_DIFFERENCE_TARGET && m_storagePressureMeasurements.size() > 1) {
          // Measure the time it took for the last 5 PSI to leak (or since last enabled if 5 PSI hasn't been leaked yet)
          m_storagePressureMeasurements.poll();
          pressureDifference = m_storagePressureMeasurements.peek().getSecond() - storagePressure;
        }

        double timeDifference = measurementTime - m_storagePressureMeasurements.peek().getFirst();
        if (timeDifference > 0.) {
          // The leak rate in PSI/minute
          leakRatePSI = pressureDifference / timeDifference;
          double averagePressure  = (m_storagePressureMeasurements.peek().getSecond() + storagePressure) / 2.;
          if (averagePressure > 0) {
            // The leak rate as a percentage of total pressure lost each minute
            leakRatePercentage = leakRatePSI / ((m_storagePressureMeasurements.peek().getSecond() + storagePressure) / 2.);
          } else {
            // Don't divide by 0
            leakRatePercentage = 0;
          }
        } else {
          // Don't divide by 0
          leakRatePSI = 0;
          leakRatePercentage = 0;
        }
      }
      else {
        // Can't measure for leaks with no measurement history
        pressureDifference = 0;
        leakRatePSI = 0;
        leakRatePercentage = 0;
      }
      m_storagePressureMeasurements.add(new Pair<Double,Double>(measurementTime, storagePressure));
    }
    SmartDashboard.putNumber("Storage Pressure", storagePressure);
    SmartDashboard.putNumber("Working Pressure", workingPressure);
    SmartDashboard.putNumber("Leak Pressure Difference PSI", pressureDifference);
    SmartDashboard.putNumber("Leak Rate PSI per minute", leakRatePSI);
    SmartDashboard.putNumber("Leak Rate Percentage per minute", leakRatePercentage);
    SmartDashboard.putBoolean("Working Pressure Warning", workingPressure < Constants.WORKING_PRESSURE_WARNING);
    SmartDashboard.putBoolean("Leak Warning", leakRatePercentage > Constants.LEAK_WARNING);
    SmartDashboard.putBoolean("Storage Pressure Sensor Disconnected", !this.isStoragePressureSensorConnected());
    SmartDashboard.putBoolean("Working Pressure Sensor Disconnected", !this.isWorkingPressureSensorConnected());
  }
}
