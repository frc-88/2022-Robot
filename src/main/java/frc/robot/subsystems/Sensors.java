/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  /**
   * Creates a new Sensors subsystem
   */
  public Sensors() {
    // start driver camera feed
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      limelight.ledOff();
    }

    navx.updateDashboard();

    // Color Sensor data
    Color detectedColor = m_colorSensor.getColor();
    double IR = m_colorSensor.getIR();

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
  }
}
