package frc.robot.util.transmission;

/**
 * /**
 * Describes a power transmission which takes motors as an input, has a sensor 
 * somehow attached to it, and can shift between 2 different gear ratios. The 
 * sensor must come after the shifting stage. Performs operations related to 
 * motor characteristics andsensor scaling.
 */
public class ShiftingTransmission extends Transmission {

    private final double m_lowInputToOutputRatio;
    private final double m_highInputToOutputRatio;
    private final double m_lowStaticFrictionVoltage;
    private final double m_highStaticFrictionVoltage;
    private final double m_lowEfficiency;
    private final double m_highEfficiency;
    private boolean m_inHighGear = false;

    /**
     * Creates a transmission object with the given physical properties.
     * 
     * Parameters:
     *  @param motor The motor that is used as an input to this transmission
     *  @param motorQuantity The number of the given motor type that are used
     *                       as an input to this transmission
     *  @param sensor The type of sensor that is attached to this transmission
     *  @param lowInputToOutputRatio The conversion to go from the input (in RPS) to
     *                               the output while in low gear. (note, 
     *                               this does not have to be a straight gear ratio. 
     *                               It can include things like converting to linear 
     *                               units on a drivetrain for the output)
     *  @param highOutputToInputRatio The conversion to go from the input (in RPS) to
     *                                the output while in high gear. (note, 
     *                                this does not have to be a straight gear ratio. 
     *                                It can include things like converting to linear 
     *                                units on a drivetrain or the output)
     *  @param outputToSensorRatio The conversion to go from the output to the rotation
     *                             of the sensor (in RPS).
     *  @param lowStaticFrictionVoltage The voltage which needs to be applied in order
     *                                  to overcome static friction in low gear
     *  @param highStaticFrictionVoltage The voltage which needs to be applied in order
     *                                   to overcome static friction in high gear
     *  @param lowEfficiency What percentage (from 0 to 1) of back-emf voltage is
     *                       actually being transferred into the velocity of the
     *                       transmission in low gear
     *  @param highEfficiency What percentage (from 0 to 1) of back-emf voltage is
     *                        actually being transferred into the velocity of the
     *                        transmission in high gear
     */
    public ShiftingTransmission(Motor motor, int motorQuantity, TransmissionSensor sensor,
            double lowInputToOutputRatio, double highInputToOutputRatio,
            double outputToSensorRatio, 
            double lowStaticFrictionVoltage, double highStaticFrictionVoltage,
            double lowEfficiency, double highEfficiency) {

        super(motor, motorQuantity, sensor, 0, outputToSensorRatio, 0, 0);
        m_lowInputToOutputRatio = lowInputToOutputRatio;
        m_highInputToOutputRatio = highInputToOutputRatio;
        m_lowStaticFrictionVoltage = lowStaticFrictionVoltage;
        m_highStaticFrictionVoltage = highStaticFrictionVoltage;
        m_lowEfficiency = lowEfficiency;
        m_highEfficiency = highEfficiency;

    }

    @Override
    protected double getInputToOutputRatio() {
        if (isInHighGear()) {
            return getHighInputToOutputRatio();
        } else {
            return getLowInputToOutputRatio();
        }
    }

    @Override
    protected double getStaticFrictionVoltage() {
        if (isInHighGear()) {
            return getHighStaticFrictionVoltage();
        } else {
            return getLowStaticFrictionVoltage();
        }
    }

    @Override
    protected double getEfficiency() {
        if (isInHighGear()) {
            return getHighEfficiency();
        } else {
            return getLowEfficiency();
        }
    }

    /**
     * Get the conversion factor from the output to the input while in low
     * gear.
     */
    protected double getLowInputToOutputRatio() {
        return m_lowInputToOutputRatio;
    }

    /**
     * Get the conversion factor from the output to the input while in high
     * gear.
     */
    protected double getHighInputToOutputRatio() {
        return m_highInputToOutputRatio;
    }

    /**
     * Get the voltage to overcome static friction in low gear.
     */
    protected double getLowStaticFrictionVoltage() {
        return m_lowStaticFrictionVoltage;
    }

    /**
     * Get the voltage to overcome static friction in low gear.
     */
    protected double getHighStaticFrictionVoltage() {
        return m_highStaticFrictionVoltage;
    }

    /**
     * Get the efficiency of the drive in low gear.
     */
    protected double getLowEfficiency() {
        return m_lowEfficiency;
    }

    /**
     * Get the efficiency of the drive in high gear.
     */
    protected double getHighEfficiency() {
        return m_highEfficiency;
    }

    /**
     * Gets the max speed of the output at the given voltage in low gear.
     */
    public double getLowMaxOutputSpeed(double voltage) {
        double backEMF = voltage - getLowStaticFrictionVoltage();
        double inputSpeed = backEMF / getMotor().getVelocityConstant() * getLowEfficiency();
        return inputSpeed * getLowInputToOutputRatio();
    }

    /**
     * Gets the max speed of the output at the given voltage in high gear.
     */
    public double getHighMaxOutputSpeed(double voltage) {
        double backEMF = voltage - getHighStaticFrictionVoltage();
        double inputSpeed = backEMF / getMotor().getVelocityConstant() * getHighEfficiency();
        return inputSpeed * getHighInputToOutputRatio();
    }

    /**
     * Are we in high gear?
     */
    public boolean isInHighGear() {
        return m_inHighGear;
    }

    /**
     * Sets the current gear ratio to the low gear ratio.
     */
    public void shiftToLow() {
        m_inHighGear = false;
    }

    /**
     * Sets the current gear ratio to the high gear ratio.
     */
    public void shiftToHigh() {
        m_inHighGear = true;
    }

}