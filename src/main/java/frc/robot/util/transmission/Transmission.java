package frc.robot.util.transmission;

import java.util.Objects;

import frc.robot.util.SyncPIDController;

/**
 * Describes a power transmission which takes motors as an input and has a 
 * sensor somehow attached to it. Performs operations related to motor 
 * characteristics and sensor scaling.
 */
public class Transmission {

    private final Motor m_motor;
    private final int m_motorQuantity;
    private final TransmissionSensor m_sensor;
    private final double m_inputToOutputRatio;
    private final double m_outputToSensorRatio;
    private final double m_staticFrictionVoltage;
    private final double m_efficiency;

    private SyncPIDController m_pidController = null;

    /**
     * Creates a transmission object with the given physical properties.
     * 
     * Parameters:
     *  @param motor The motor that is used as an input to this transmission
     *  @param motorQuantity The number of the given motor type that are used
     *                       as an input to this transmission
     *  @param sensor The type of sensor that is attached to this transmission
     *  @param inputToOutputRatio The conversion to go from the input (in RPS) to
     *                            the output. (note, this does not have to be a
     *                            straight gear ratio. It can include things like
     *                            converting to linear units on a drivetrain
     *                            for the output)
     *  @param outputToSensorRatio The conversion to go from the rotation of
     *                             the output to the sensor (in RPS).
     *  @param staticFrictionVoltage The voltage which needs to be applied in order
     *                               to overcome static friction
     *  @param efficiency What percentage (from 0 to 1) of back-emf voltage is
     *                    actually being transferred into the velocity of the
     *                    transmission
     */
    public Transmission(Motor motor, int motorQuantity, TransmissionSensor sensor,
            double inputToOutputRatio, double outputToSensorRatio, 
            double staticFrictionVoltage, double efficiency) {
        
        m_motor = motor;
        m_motorQuantity = motorQuantity;
        m_sensor = sensor;
        m_inputToOutputRatio = inputToOutputRatio;
        m_outputToSensorRatio = outputToSensorRatio;
        m_staticFrictionVoltage = staticFrictionVoltage;
        m_efficiency = efficiency;
    }

    /**
     * Get the motor used by this transmission.
     */
    protected Motor getMotor() {
        return m_motor;
    }

    /**
     * Gets the number of motors on this transmission.
     */
    protected int getMotorQuantity() {
        return m_motorQuantity;
    }

    /**
     * Gets the sensor used by this transmission.
     */
    protected TransmissionSensor getSensor() {
        return m_sensor;
    }

    /**
     * Get the conversion factor from the input to the output.
     */
    protected double getInputToOutputRatio() {
        return m_inputToOutputRatio;
    }
    /**
     * Get the conversion factor from the output to the sensor.
     */
    protected double getOutputToSensorRatio() {
        return m_outputToSensorRatio;
    }
    /**
     * Get the conversion factor from the input to the sensor.
     */
    protected double getInputToSensorRatio() {
        return getInputToOutputRatio() * getOutputToSensorRatio();
    }


    /**
     * Get the voltage to overcome static friction.
     */
    protected double getStaticFrictionVoltage() {
        return m_staticFrictionVoltage;
    }
    /**
     * Get the efficiency of the drive.
     */
    protected double getEfficiency() {
        return m_efficiency;
    }


    /**
     * Convert a position reading from the sensor to the output.
     */
    public double convertSensorPositionToOutput(double sensorPos) {
        return getSensor().convertNativePositionUnitsToRotations(sensorPos)
                / getOutputToSensorRatio();
    }
    /**
     * Convert a position reading from the output to the sensor.
     */
    public double convertOutputPositionToSensor(double outputPos) {
        return getSensor().convertRotationsToNativePositionUnits(
                outputPos * getOutputToSensorRatio());
    }

    /**
     * Convert a velocity reading from the sensor to the output.
     */
    public double convertSensorVelocityToOutput(double sensorVel) {
        return getSensor().convertNativeVelocityUnitsToRPS(sensorVel)
                / getOutputToSensorRatio();
    }
    /**
     * Convert a velocity reading from the output to the sensor.
     */
    public double convertOutputVelocityToSensor(double outputVel) {
        return getSensor().convertRPSToNativeVelocityUnits(
                outputVel * getOutputToSensorRatio());
    }
    /**
     * Convert a velocity from the output to the input.
     */
    protected double convertOutputVelocityToInput(double outputVel) {
        return outputVel / getInputToOutputRatio();
    }
    /**
     * Convert a velocity from the input to the output.
     */
    protected double convertInputVelocityToOutput(double inputVel) {
        return inputVel * getInputToOutputRatio();
    }
    /**
     * Convert a velocity reading from the sensor to the input.
     */
    protected double convertSensorVelocityToInput(double sensorVel) {
        return getSensor().convertNativeVelocityUnitsToRPS(sensorVel)
                / getInputToSensorRatio();
    }
    /**
     * Convert a velocity reading from the input to the sensor.
     */
    protected double convertInputVelocityToSensor(double inputVel) {
        return getSensor().convertRPSToNativeVelocityUnits(
                inputVel * getInputToSensorRatio());
    }

    /**
     * Gets the max speed of the output at the given voltage.
     */
    public double getMaxOutputSpeed(double voltage) {
        double backEMF = voltage - getStaticFrictionVoltage();
        double inputSpeed = backEMF / getMotor().getVelocityConstant() * getEfficiency();
        return convertInputVelocityToOutput(inputSpeed);
    }


    /**
     * Get the required back-emf voltage to drive this transmission to the
     * given motor velocity.
     */
    protected double getBackEMF(double motorSpeed) {
        return (motorSpeed / getEfficiency()) * getMotor().getVelocityConstant();
    }

    /**
     * Get the feedforward voltage to set the output to the given speed.
     */
    public double getFeedforwardVoltage(double outputSpeed) {
        return getBackEMF(convertOutputVelocityToInput(outputSpeed))
                + Math.signum(outputSpeed) * getStaticFrictionVoltage();
    }

    /**
     * Get the current draw from the motor at the given commanded voltage and
     * sensor velocity.
     */
    public double getCurrentDraw(double voltage, double sensorVelocity) {
        double motorVelocity = convertSensorVelocityToInput(sensorVelocity);
        double backEMF = getBackEMF(motorVelocity);
        double windingsVoltage = voltage - backEMF;
        double currentDrawPerMotor = Math.signum(voltage) * windingsVoltage 
                / getMotor().getWindingsResistance();
        return Math.max(currentDrawPerMotor * getMotorQuantity(), 0);
    }

    /**
     * Sets a PID controller to be used for velocity-to-commanded-voltage 
     * calculations (current just the getCurrentLimitedVoltage() method). 
     * Input is the output velocity, output is volts. Set to null if there 
     * shouldn't be a PID controller used.
     */
    public void setVelocityPID(SyncPIDController controller) {
        this.m_pidController = controller;
    }

    /**
     * Gets the voltage to command right now if trying to drive the output to the
     * given velocity without exceeding the given current limit.
     * 
     * Parameters:
     * @param targetVelocity The desired velocity at the output of the transmission
     * @param currentSensorVelocity The current velocity reading from the sensor
     * @param currentLimit The maximum current, in amps, that we want the sum
     *                     of all the motors to draw
     */
    public double getCurrentLimitedVoltage(double targetVelocity, double currentSensorVelocity,
            double currentLimit) {

        // Determine the target voltage to get the requested velocity
        double targetMotorVelocity = convertOutputVelocityToInput(targetVelocity);
        double targetVoltage = getBackEMF(targetMotorVelocity) 
                + Math.signum(targetMotorVelocity) * getStaticFrictionVoltage();

        // Apply the PID on the voltage
        double currentOutputVelocity = 
                convertSensorVelocityToOutput(currentSensorVelocity);
        if (Objects.nonNull(m_pidController) && Math.abs(targetVoltage) >= 0.01) {
            targetVoltage += 
                m_pidController.calculateOutput(currentOutputVelocity, targetVelocity);
        }
        
        // Get the current back-emf voltage
        double currentMotorVelocity = convertSensorVelocityToInput(currentSensorVelocity);
        double currentBackEMF = getBackEMF(currentMotorVelocity);

        // If we are deccelerating, we don't need to worry about current draw
        if (targetVoltage == 0 
                || (Math.signum(targetVoltage) == Math.signum(currentBackEMF) 
                    && Math.abs(targetVoltage) < Math.abs(currentBackEMF))) {

            return targetVoltage;

        }

        // If we are changing directions, commanding 0 won't cause any current
        // draw, but commanding anything past it will cause the motor's
        // connection to the battery to be reversed, and we will see a lot of
        // current draw. See if that will be too much
        if (Math.signum(targetVoltage) != Math.signum(currentBackEMF)
                && getCurrentDraw(Math.signum(targetVoltage) * 0.01, 
                    currentSensorVelocity) > currentLimit) {

            return 0;

        }

        // Determine the voltage across the motor windings to get the specified 
        // current limit
        double maxWindingsVoltage = (currentLimit / getMotorQuantity())
                 * getMotor().getWindingsResistance();
        
        // Determine the voltage to command
        if (targetVoltage > 0) {
            return Math.min(targetVoltage, currentBackEMF + maxWindingsVoltage);
        }  else {
            return Math.max(targetVoltage, currentBackEMF - maxWindingsVoltage);
        }

    }

    /**
     * Gets the expected current draw if the given velocity was to be achieved.
     * 
     * Parameters:
     * @param targetVelocity The desired velocity at the output of the transmission
     * @param currentSensorVelocity The current velocity reading from the sensor
     */
    public double getExpectedCurrentDraw(double targetVelocity, double currentSensorVelocity) {

        // Determine the target voltage to get the requested velocity
        double targetMotorVelocity = convertOutputVelocityToInput(targetVelocity);
        double targetVoltage = getBackEMF(targetMotorVelocity) 
                + Math.signum(targetMotorVelocity) * getStaticFrictionVoltage();
        
        // Get the current back-emf voltage
        double currentMotorVelocity = convertSensorVelocityToInput(currentSensorVelocity);
        double currentBackEMF = getBackEMF(currentMotorVelocity);

        // If we are deccelerating, there is no current draw
        if (targetVoltage == 0 
                || (Math.signum(targetVoltage) == Math.signum(currentBackEMF) 
                    && Math.abs(targetVoltage) < Math.abs(currentBackEMF))) {

            return 0;
        } else {
            return getMotorQuantity() * Math.abs((targetVoltage - currentBackEMF)) / getMotor().getWindingsResistance();
        }

    }

}