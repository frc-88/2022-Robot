package frc.robot.util.transmission;

/**
 * Contains conversion information for a Grayhill encoder connected to a Talon.
 */
public class TalonGrayhill implements TransmissionSensor {

    // TODO: CONFIRM THIS
    private static final int TICKS_PER_ROTATION = 512;
    private static final double NATIVE_TIME_IN_SECONDS = 100. / 1000.; // 100ms

    @Override
    public double convertRotationsToNativePositionUnits(double rotations) {
        return rotations * TICKS_PER_ROTATION;
    }

    @Override
    public double convertNativePositionUnitsToRotations(double position) {
        return position / TICKS_PER_ROTATION;
    }

    @Override
    public double convertRPSToNativeVelocityUnits(double rpm) {
        return convertRotationsToNativePositionUnits(rpm) * NATIVE_TIME_IN_SECONDS;
    }

    @Override
    public double convertNativeVelocityUnitsToRPS(double velocity) {
        return convertNativePositionUnitsToRotations(velocity) / NATIVE_TIME_IN_SECONDS;
    }

}