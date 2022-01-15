package frc.robot.driveutil;

import java.lang.Math;
import java.util.function.DoubleSupplier;

public class DriveUtils{

    public static double signedPow(double base, int exp){
        double value = 0;
        
        if(base < 0 && exp%2==0){
            value = -Math.pow(base,exp);
        }
        else{
            value = Math.pow(base,exp);
        }

        return value;
    }
    
    public static double deadbandExponential(double spd, int exp, double deadband) {
        return Math.abs(spd)<deadband ? 0 : DriveUtils.signedPow(spd - deadband*Math.signum(spd), exp) / (1 - deadband);
    }

    public static DoubleSupplier deadbandExponential(DoubleSupplier input, int exp, double deadband) {
        return () -> deadbandExponential(input.getAsDouble(), exp, deadband);
    }

    public static double cheesyTurn(double spd, double turnRate, double minTurn, double maxTurn) {
        spd = Math.abs(spd);

        if (Math.abs(spd) < minTurn) {
            spd = minTurn;
        }
        return spd * turnRate * maxTurn;
    }

    public static DoubleSupplier cheesyTurn(DoubleSupplier speed, DoubleSupplier turnRate, double minTurn, double maxTurn) {
        return () -> cheesyTurn(speed.getAsDouble(), turnRate.getAsDouble(), minTurn, maxTurn);
    }

}