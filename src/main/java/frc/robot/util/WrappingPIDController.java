package frc.robot.util;

import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class WrappingPIDController extends SyncPIDController {

    private double max;
    private double min;

    public WrappingPIDController(double max, double min, double kP, double kI, double kD, double kF, double iZone, double iMax, double tolerance) {
        super(kP, kI, kD, kF, iZone, iMax, tolerance);
        this.max = max;
        this.min = min;
    }

    public WrappingPIDController(double max, double min, double kP, double kI, double kD) {
        super(kP, kI, kD);
        this.max = max;
        this.min = min;
    }

    public WrappingPIDController(double max, double min, PIDPreferenceConstants constants) {
        super(constants);
        this.max = max;
        this.min = min;
    }

    @Override
    public double calculateOutput(double input, double setpoint) {
        if (input > setpoint && ((max - input) + (setpoint - min) < (input - setpoint))) {
            setpoint += max - min;
        } else if (input < setpoint && ((max - setpoint) + (input - min) < (setpoint - input))) {
            input += max - min;
        }
        return super.calculateOutput(input, setpoint);
    }

}