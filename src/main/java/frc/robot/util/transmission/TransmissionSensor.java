package frc.robot.util.transmission;

/**
 * Specification for the information which a sensor on a transmission object needs
 * to provide.
 */
public interface TransmissionSensor {

    /**
     * Convert from rotations into this sensor's native position units.
     */
    public double convertRotationsToNativePositionUnits(double rotations);

    /**
     * Convert from this sensor's native position units into rotations.
     */
    public double convertNativePositionUnitsToRotations(double position);

    /**
     * Convert from RPS to this sensor's native velocity units.
     */
    public double convertRPSToNativeVelocityUnits(double rpm);

    /**
     * Conver from this sensor's native velocity units to RPS.
     */
    public double convertNativeVelocityUnitsToRPS(double velocity);

}