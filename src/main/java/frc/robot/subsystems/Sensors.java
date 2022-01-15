/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.NavX;

/**
 * we gather data
 * about the world around us
 * to make good choices
 */

public class Sensors extends SubsystemBase {
  private final NavX m_navx = new NavX();
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(Constants.I2C_ONBOARD);

  private double m_yawOffset = 0.0;

  /**
   * Creates a new Sensors subsystem
   */
  public Sensors() {
  }

  public void zeroYaw() {
    m_yawOffset = m_navx.getYaw();
  }

  public double getYaw() {
    return m_navx.getYaw() - m_yawOffset;
  }

  public double getYawRate() {
    return m_navx.getYawRate();
  } 

  @Override
  public void periodic() {
    // NavX data
    SmartDashboard.putNumber("NavX Yaw", getYaw());
    SmartDashboard.putNumber("NavX Yaw Rate", getYawRate());
    SmartDashboard.putNumber("NavX Pitch", m_navx.getPitch());
    SmartDashboard.putNumber("NavX Roll", m_navx.getRoll());

    // Color Sensor data
    Color detectedColor = m_colorSensor.getColor();
    double IR = m_colorSensor.getIR();

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
  }
}
