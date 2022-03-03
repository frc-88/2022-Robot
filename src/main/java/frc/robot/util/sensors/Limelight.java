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

    private final DoublePreferenceConstant p_heightHoodUp = new DoublePreferenceConstant("Limelight Height Up", 42.801723);
    private final DoublePreferenceConstant p_angleHoodUp = new DoublePreferenceConstant("Limelight Angle Up", 0);
    private final DoublePreferenceConstant p_radiusHoodUp = new DoublePreferenceConstant("Limelight Radius Up", 6.0);
    private final DoublePreferenceConstant p_heightHoodDown = new DoublePreferenceConstant("Limelight Height Down", 37.769);
    private final DoublePreferenceConstant p_angleHoodDown = new DoublePreferenceConstant("Limelight Angle Down", 0);
    private final DoublePreferenceConstant p_radiusHoodDown = new DoublePreferenceConstant("Limelight Radius Down", 8.0);
    private final DoublePreferenceConstant p_targetThreshold = new DoublePreferenceConstant("Limelight Target Threshold", 0);
    private final DoublePreferenceConstant p_testDistance = new DoublePreferenceConstant("Limelight Test Distance", 0);

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

    private double getLimelightHeight() {
        return (m_hoodUp ? p_heightHoodUp : p_heightHoodDown).getValue();
    }

    private double getLimelightAngle() {
        return (m_hoodUp ? p_angleHoodUp : p_angleHoodDown).getValue();
    }

    /**
     * Calculate the distance to the target.
     * 
     * https://www.chiefdelphi.com/t/calculating-distance-to-vision-target/387183/6
     * has details regarding the cosine term, used to adjust for when
     * the target isn't in the center of the field of vision.
     * 
     * @return The distance to the target
     */
    public double calcDistanceToTarget() {
        double distance = 0;

        if (isConnected() && hasTarget()) {
            distance = (Constants.FIELD_VISION_TARGET_HEIGHT - getLimelightHeight()) /
                    (Math.tan(Math.toRadians(getLimelightAngle() + getTargetVerticalOffsetAngle()))
                            * Math.cos(Math.toRadians(getTargetHorizontalOffsetAngle())));
        }

        return distance;
    }

    /**
     * Calculate the mount angle of the limelight assuming the target
     * is the distance specified by the "Limelight Test Distance" value.
     * Useful during field calibration.
     * 
     * @return The mount angle of the limelight in degrees
     */
    public double calcLimelightAngle() {
        return Math.toDegrees(Math.atan((Constants.FIELD_VISION_TARGET_HEIGHT - getLimelightHeight())
                / (p_testDistance.getValue() * Math.cos(Math.toRadians(getTargetHorizontalOffsetAngle())))))
                - getTargetVerticalOffsetAngle();
    }

    public double calcTurretOffset() {
        double distance = calcDistanceToTarget();
        double angle = Math.toRadians(getTargetHorizontalOffsetAngle());

        return hasTarget() ? Math.toDegrees(Math.atan((distance * Math.sin(angle)) /
            (distance * Math.cos(angle) - (isHoodUp()?p_radiusHoodUp.getValue():p_radiusHoodDown.getValue())))) : 0.0;
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
