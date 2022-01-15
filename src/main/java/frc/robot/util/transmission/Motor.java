package frc.robot.util.transmission;

/**
 * Specification for the information which a motor being used as an input to a
 * gearbox needs to provide.
 */
public interface Motor {

    /**
     * Get the conversion rate from motor speed in rotations per second to
     * back-emf voltage.
     */
    public double getVelocityConstant();

    /**
     * Get the resistance across the windings for this motor, in Ohms.
     */
    public double getWindingsResistance();

}