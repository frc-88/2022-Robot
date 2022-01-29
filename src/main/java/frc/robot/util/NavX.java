/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/**
 * Add your docs here.
 */
public class NavX {
    private final AHRS m_ahrs;
    private double yawOffset;

    public NavX() {
        m_ahrs = new AHRS(SPI.Port.kMXP);
    }

    public void zeroYaw() {
        yawOffset = getYaw() + yawOffset;
    }

    public double getYaw() {
        return m_ahrs.getYaw() + yawOffset;
    }

    public double getYawRate() {
        return Math.toDegrees(m_ahrs.getRate());
    }

    // 1-Jan-2020 Adding Pitch and Roll functions
    // At startup, the NavX will calibrate.  It is necessary to set yaw to zero, 
    // but it does not appear that pitch and roll need to be zeroed.

    //Pitch is the slope of the line relative to horizontal.
    //Think of an airplane nose that is either up or down.
    public float getPitch() {
        //Standard method
        return m_ahrs.getPitch();
    }

    //Roll is the rotation around the center axis.
    //Think of an airplane's wings moving around it's body.
    public float getRoll() {
        //Standard method
        return m_ahrs.getRoll();
    }
}
