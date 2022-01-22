package frc.robot.util.transmission;

/**
 * Contains motor characteristic information for the 775 Pro motor.
 */
public class Vex775Pro implements Motor {

    @Override
    public double getVelocityConstant() {
        return 12. / 18730. * 60.;
    }

    @Override
    public double getWindingsResistance() {
        return 12. / 134.;
    }
    
}