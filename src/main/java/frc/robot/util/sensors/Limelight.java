/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util.sensors;

import edu.wpi.first.networktables.*;

/**
 * Limelight wrapper class
 */
public class Limelight {
    private String m_name;

    private NetworkTable m_table;
    private NetworkTableEntry m_ta;
    private NetworkTableEntry m_tv;
    private NetworkTableEntry m_tx;
    private NetworkTableEntry m_ty;
    private NetworkTableEntry m_ts;
    private NetworkTableEntry m_camtran;
    private NetworkTableEntry m_ledMode;
    private NetworkTableEntry m_stream;
    private NetworkTableEntry m_camMode;
    private NetworkTableEntry m_pipeline;
    private NetworkTableEntry m_getpipe;

    private boolean m_ledOn;

    /**
     * Construct a Limelight instance with the default NetworkTables table name.
     */
    public Limelight() {
        this("limelight");
    }

    public Limelight(String network_table_name) {
        m_name = network_table_name;
        m_table = NetworkTableInstance.getDefault().getTable(network_table_name);
        m_pipeline = m_table.getEntry("pipeline");
        m_getpipe = m_table.getEntry("getpipe");
        m_tv = m_table.getEntry("tv");
        m_ta = m_table.getEntry("ta");
        m_tx = m_table.getEntry("tx");
        m_ty = m_table.getEntry("ty");
        m_ts = m_table.getEntry("ts");
        m_camtran = m_table.getEntry("camtran");
        m_ledMode = m_table.getEntry("ledMode");
        m_stream = m_table.getEntry("stream");
        m_camMode = m_table.getEntry("camMode");
        m_ledOn = false;

        m_stream.setNumber(0);
        setPipeline(0);
    }

    public String getName() {
        return m_name;
    }

    /**
     * @return True if data is being received from the Limelight.
     */
    public boolean isConnected() {
        return (m_ta.exists() && m_tv.exists() && m_tx.exists() && m_ty.exists());
    }

    /**
     * @return True if the Limelight has a recognized target.
     */
    public boolean hasTarget() {
        return m_ledOn && (m_tv.getDouble(0.0) == 1.0);
    }

    /**
     * Get the horizontal offset angle of the target from the center of the camera
     * frame. If no target is seen, returns zero.
     * 
     * @return A measurement in degrees in the range [-27, 27]
     */
    public double getTargetHorizontalOffsetAngle() {
        return hasTarget() ? m_tx.getDouble(0.0) : 0.0;
    }

    /**
     * Get the vertical offset angle of the target from the center of the camera
     * frame. If no target is seen, returns zero.
     * 
     * @return A measurement in degrees in the range [-20.5, 20.5]
     */
    public double getTargetVerticalOffsetAngle() {
        return hasTarget() ? m_ty.getDouble(0.0) : 0.0;
    }

    /**
     * Get the area of the target as a percentage of the total camera frame. If no
     * target is seen, returns zero.
     * 
     * @return A percentage in the range of [0, 1]
     */
    public double getTargetArea() {
        return hasTarget() ? m_ta.getDouble(0.0) : 0.0;
    }

    /**
     * Get the skew or rotation
     * 
     * @return -90 degrees to 0 degrees
     */
    public double getTargetSkew() {
        return hasTarget() ? m_ts.getDouble(0.0) : 0.0;
    }

    // LED control
    public void ledPipeline() {
        m_ledMode.setNumber(0);
    }

    public void ledOff() {
        m_ledOn = false;
        m_ledMode.setNumber(1);
    }

    public void ledBlink() {
        m_ledMode.setNumber(2);
    }

    public void ledOn() {
        m_ledOn = true;
        m_ledMode.setNumber(3);
    }

    // camera feed control
    public void camVision() {
        m_camMode.setNumber(0);
    }

    public void camDriver() {
        m_camMode.setNumber(1);
    }

    public void setPipeline(double pipeline) {
        m_pipeline.setDouble(pipeline);
    }

    public double getPipeline() {
        return m_getpipe.getDouble(0);
    }

    public void setPip() {
        m_stream.setNumber(2);
    }

}
