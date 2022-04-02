/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util.sensors;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.*;
import frc.robot.Constants;
import frc.robot.util.ValueInterpolator;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.IntPreferenceConstant;

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
    private NetworkTableEntry m_ledMode;
    private NetworkTableEntry m_stream;
    private NetworkTableEntry m_camMode;
    private NetworkTableEntry m_pipeline;
    private NetworkTableEntry m_getpipe;

    private MedianFilter m_txFilter;
    private MedianFilter m_tyFilter;

    private double m_targetHorizontalOffsetAngle;
    private double m_targetVerticalOffsetAngle;
    private double m_targetDistance;
    private double m_turretOffset;
    private double m_calibrationAngle;

    private double m_motionOffset;

    private final DoublePreferenceConstant p_height = new DoublePreferenceConstant("Limelight Height", 42.801723);
    private final DoublePreferenceConstant p_angle = new DoublePreferenceConstant("Limelight Angle", 0);
    private final DoublePreferenceConstant p_radius = new DoublePreferenceConstant("Limelight Radius", 6.0);
    private final DoublePreferenceConstant p_targetThreshold = new DoublePreferenceConstant("Limelight Target Threshold", 0);
    private final DoublePreferenceConstant p_testDistance = new DoublePreferenceConstant("Limelight Test Distance", 0);
    private final IntPreferenceConstant p_filterSize = new IntPreferenceConstant("Limelight Filter Size", 10);

    private final ValueInterpolator m_tofInterpolator = new ValueInterpolator(
        new ValueInterpolator.ValuePair(6.0, 1),
        new ValueInterpolator.ValuePair(12.0, 2),
        new ValueInterpolator.ValuePair(24.0, 3));
        
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
        m_ledMode = m_table.getEntry("ledMode");
        m_stream = m_table.getEntry("stream");
        m_camMode = m_table.getEntry("camMode");

        m_stream.setNumber(0);
        setPipeline(0);

        m_txFilter = new MedianFilter(p_filterSize.getValue());
        m_tyFilter = new MedianFilter(p_filterSize.getValue());
    }

    public void periodic() {
        updateTargetHorizontalOffsetAngle();
        updateTargetVerticalOffsetAngle();
        m_targetDistance = calcDistanceToTarget();
        m_calibrationAngle = calcCalibrationAngle();
        m_turretOffset = calcTurretOffset();
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
        return hasTarget() && (Math.abs(m_targetHorizontalOffsetAngle + m_motionOffset) < p_targetThreshold.getValue());
    }

    public double getTargetHorizontalOffsetAngle() {
        return m_targetHorizontalOffsetAngle;
    }

    public double getTargetVerticalOffsetAngle() {
        return m_targetVerticalOffsetAngle;
    }

    public double getTargetDistance() {
        return m_targetDistance;
    }

    public double getTurretOffset() {
        return m_turretOffset;
    }

    public double getCalibrationAngle() {
        return m_calibrationAngle;
    }

    public void setMotionOffset(double motionOffset) {
        m_motionOffset = motionOffset;
    }

    public double getMotionOffset() {
        return m_motionOffset;
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
    private double calcDistanceToTarget() {
        double distance = 0.0;

        if (isConnected() && hasTarget()) {
            distance = (Constants.FIELD_VISION_TARGET_HEIGHT - p_height.getValue()) /
                    (Math.tan(Math.toRadians(p_angle.getValue() + m_targetVerticalOffsetAngle))
                            * Math.cos(Math.toRadians(m_targetHorizontalOffsetAngle)));
        }

        return distance;
    }

    /**
     * Calculate the mount angle of the limelight assuming the target
     * is the distance specified by the "Limelight Test Distance" preference
     * value. Useful during field calibration.
     * 
     * @return The mount angle of the limelight in degrees
     */
    private double calcCalibrationAngle() {
        return Math.toDegrees(Math.atan((Constants.FIELD_VISION_TARGET_HEIGHT - p_height.getValue())
                / (p_testDistance.getValue() * Math.cos(Math.toRadians(m_targetHorizontalOffsetAngle)))))
                - m_targetVerticalOffsetAngle;
    }

    /**
     * Calculate the degrees to rotate the turret in order to focus on the
     * target. Distance calculations need to be accurate!
     * 
     * If we don't have accurate distance measurements, a proportional
     * constant could probably be used and work just as well. 
     * 
     * @return The mount angle of the limelight in degrees
     */
    private double calcTurretOffset() {
        double angle = Math.toRadians(m_targetHorizontalOffsetAngle);

        return hasTarget() ? Math.toDegrees(Math.atan(Math.sin(angle)/(Math.cos(angle) + (p_radius.getValue()/m_targetDistance)))) + m_motionOffset : 0.0;
    }

    /**
     * 
     * @param robotSpeed in feet per second
     * @return
     */
    public double calcMovingDistance(double robotSpeed, double turretAngle) {
        double target = m_targetDistance / 12.0;
        double tof;

        for (int i = 0; i < 3; i++) {
            tof = m_tofInterpolator.getInterpolatedValue(target);
            target = Math.sqrt(
                Math.pow(m_targetDistance/12.0, 2) 
                + (robotSpeed * tof) 
                - (2 * m_targetDistance/12.0 * robotSpeed * tof * 
                    Math.cos(180 + m_turretOffset - turretAngle)
                )
            );
        }

        return target;
    }

    /**
     * 
     * @param robotSpeed in feet per second
     * @return
     */
    public double calcMovingTurretOffset(double robotSpeed, double turretAngle) {
        double offset = 0.0;
        double distance = calcMovingDistance(robotSpeed, turretAngle);
        double tof = m_tofInterpolator.getInterpolatedValue(distance);

        Math.asin(Math.sin(180 + m_turretOffset - turretAngle) *
            robotSpeed * tof / distance);

        return offset;
    }
    
    /**
     * Get the horizontal offset angle of the target from the center of the camera
     * frame. If no target is seen, returns zero.
     * 
     * @return A measurement in degrees in the range [-27, 27]
     */
    private void updateTargetHorizontalOffsetAngle() {
        double next = hasTarget() ? m_tx.getDouble(0.0) : 0.0;
        m_targetHorizontalOffsetAngle = m_txFilter.calculate(next);
    }

    /**
     * Get the vertical offset angle of the target from the center of the camera
     * frame. If no target is seen, returns zero.
     * 
     * @return A measurement in degrees in the range [-20.5, 20.5]
     */
    private void updateTargetVerticalOffsetAngle() {
        double next = hasTarget() ? m_ty.getDouble(0.0) : 0.0;
        m_targetVerticalOffsetAngle = m_tyFilter.calculate(next);
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
