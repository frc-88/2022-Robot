/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.sensors.Limelight;
import frc.robot.util.sensors.NavX;
import frc.robot.util.sensors.REVColorSensor;

/**
 * we gather data
 * about the world around us
 * to make good choices
 */

public class Sensors extends SubsystemBase {
  public final NavX navx = new NavX();
  public final Limelight limelight = new Limelight();
  public final REVColorSensor colorSensor = new REVColorSensor();

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
    colorSensor.updateDashboard();
  }
}
