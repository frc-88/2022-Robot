/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util.sensors;

import edu.wpi.first.networktables.*;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

/**
 * Limelight wrapper class
 */
public class Limelight {
    private String m_name;

    private NetworkTable m_table;
    private boolean m_hoodUp;
    private NetworkTableEntry m_ta;
    private NetworkTableEntry m_tv;
    private NetworkTableEntry m_tx;
    private NetworkTableEntry m_ty;
    private NetworkTableEntry m_ts;
    private NetworkTableEntry m_ledMode;
    private NetworkTableEntry m_stream;
    private NetworkTableEntry m_camMode;
    private NetworkTableEntry m_pipeline;
    private NetworkTableEntry m_getpipe;

    private final DoublePreferenceConstant p_heightHoodUp = new DoublePreferenceConstant("Limelight Height Up", Constants.LIMELIGHT_HEIGHT_HOOD_UP_DFT);
    private final DoublePreferenceConstant p_angleHoodUp = new DoublePreferenceConstant("Limelight Angle Up", Constants.LIMELIGHT_ANGLE_HOOD_UP_DFT);
    private final DoublePreferenceConstant p_heightHoodDown = new DoublePreferenceConstant("Limelight Height Down", Constants.LIMELIGHT_ANGLE_HOOD_DOWN_DFT);
    private final DoublePreferenceConstant p_angleHoodDown = new DoublePreferenceConstant("Limelight Angle Down", Constants.LIMELIGHT_ANGLE_HOOD_DOWN_DFT);
    private final DoublePreferenceConstant p_targetThreshold = new DoublePreferenceConstant("Limelight Target Threshold", Constants.SHOOTER_LIMELIGHT_THRESHOLD);
    private final DoublePreferenceConstant p_testDistance = new DoublePreferenceConstant("Limelight Test Distance", Constants.SHOOTER_LIMELIGHT_THRESHOLD);

    /**
     * Construct a Limelight instance with the default NetworkTables table name.
     */
    public Limelight() {
        this("limelight");
    }

    public Limelight(String network_table_name) {
        m_name = network_table_name;
        m_hoodUp = false;
        m_table = NetworkTableInstance.getDefault().getTable(network_table_name);
        m_pipeline = m_table.getEntry("pipeline");
        m_getpipe = m_table.getEntry("getpipe");
        m_tv = m_table.getEntry("tv");
        m_ta = m_table.getEntry("ta");
        m_tx = m_table.getEntry("tx");
        m_ty = m_table.getEntry("ty");
        m_ts = m_table.getEntry("ts");
        m_ledMode = m_table.getEntry("ledMode");
        m_stream = m_table.getEntry("stream");
        m_camMode = m_table.getEntry("camMode");

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
        return isLightOn() && (m_tv.getDouble(0.0) == 1.0);
    }

    public boolean onTarget() {
        return hasTarget() && (Math.abs(getTargetHorizontalOffsetAngle()) < p_targetThreshold.getValue());
    }

    public void setHood(boolean hoodUp) {
        m_hoodUp = hoodUp;
    }

    public boolean isHoodUp() {
        return m_hoodUp;
    }

    public double getDistanceToTarget() {
        double distance = 0;
        double height = m_hoodUp ? p_heightHoodUp.getValue() : p_heightHoodDown.getValue();
        double angle = m_hoodUp ? p_angleHoodUp.getValue() : p_angleHoodDown.getValue();

        if (isConnected() && hasTarget()) {
            double ty = getTargetVerticalOffsetAngle();
    
          distance = (Constants.FIELD_VISION_TARGET_HEIGHT - height) / 
             Math.tan(Math.toRadians(angle + ty));
        }
    
        return distance;
      }
    
    public double calcLimelightAngle() {
        return Math.toDegrees(Math.atan((Constants.FIELD_VISION_TARGET_HEIGHT - (m_hoodUp ? p_heightHoodUp : p_heightHoodDown).getValue()) / p_testDistance.getValue())) - getTargetVerticalOffsetAngle();
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
        m_ledMode.setNumber(1);
    }

    public void ledBlink() {
        m_ledMode.setNumber(2);
    }

    public void ledOn() {
        m_ledMode.setNumber(3);
    }

    public boolean isLightOn() {
        return (m_ledMode.getDouble(1.0) != 1.0);
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
