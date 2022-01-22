// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.sensors;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class REVColorSensor {
    private final ColorSensorV3 m_colorSensor;

    public REVColorSensor() {
        this(I2C.Port.kOnboard);
    }

    public REVColorSensor(Port port) {
        m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    }

    public void updateDashboard() {
        // Color Sensor data
        Color detectedColor = m_colorSensor.getColor();
        double IR = m_colorSensor.getIR();

        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);
    }

}
