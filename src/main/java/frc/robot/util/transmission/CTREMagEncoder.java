/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util.transmission;

/**
 * Add your docs here.
 */
public class CTREMagEncoder implements TransmissionSensor{
    
    private static final int TICKS_PER_ROTATION = 4096;
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
