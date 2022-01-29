// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.sensors;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class REVColorSensor {
    private final ColorSensorV3 m_colorSensor;

    public REVColorSensor() {
        this(I2C.Port.kOnboard);
    }

    public REVColorSensor(Port port) {
        m_colorSensor = new ColorSensorV3(port);
    }

    public boolean hasCargo() {
        return (m_colorSensor.getProximity() < Constants.COLOR_SENSOR_PROXIMITY_THRESHOLD);
    }

    public boolean isCargoOurs() {
        Color detectedColor = m_colorSensor.getColor();

        return hasCargo() && ((DriverStation.getAlliance() == DriverStation.Alliance.Blue) == 
            ((detectedColor.blue > Constants.COLOR_SENSOR_BLUE_CARGO_BLUE_THRESHOLD) &&
             (detectedColor.red < Constants.COLOR_SENSOR_BLUE_CARGO_RED_THRESHOLD) &&
             (detectedColor.green < Constants.COLOR_SENSOR_BLUE_CARGO_GREEN_THRESHOLD)));
    }

    public void periodic() {
        // Color Sensor data
        Color detectedColor = m_colorSensor.getColor();
        SmartDashboard.putNumber("ColorSensor:Red", detectedColor.red);
        SmartDashboard.putNumber("ColorSensor:Green", detectedColor.green);
        SmartDashboard.putNumber("ColorSensor:Blue", detectedColor.blue);
        SmartDashboard.putNumber("ColorSensor:IR", m_colorSensor.getIR());
        SmartDashboard.putNumber("ColorSensor:Proximity", m_colorSensor.getProximity());

        SmartDashboard.putBoolean("ColorSensor:HasCargo?", hasCargo());
        SmartDashboard.putBoolean("ColorSensor:IsCargoOurs?", isCargoOurs());
    }

}
