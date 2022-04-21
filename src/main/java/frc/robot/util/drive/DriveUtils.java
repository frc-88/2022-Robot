package frc.robot.util.drive;

import java.lang.Math;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

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
        return Math.abs(spd)<deadband ? 0 : DriveUtils.signedPow(spd - deadband*Math.signum(spd), exp) / signedPow(1 - deadband, exp);
    }

    public static DoubleSupplier deadbandExponential(DoubleSupplier input, int exp, double deadband) {
        return () -> deadbandExponential(input.getAsDouble(), exp, deadband);
    }

    public static double cheesyTurn(double spd, double turnRate, double minTurn, double maxTurn) {
        spd = Math.abs(spd);

        if (Math.abs(spd) < minTurn) {
            spd = (Math.abs(spd) < 0.001) ? minTurn : Math.signum(spd) * minTurn;
        }
        return spd * turnRate * maxTurn;
    }

    public static DoubleSupplier cheesyTurn(DoubleSupplier speed, DoubleSupplier turnRate, double minTurn, double maxTurn) {
        return () -> cheesyTurn(speed.getAsDouble(), turnRate.getAsDouble(), minTurn, maxTurn);
    }

    public static double mod(double base, int modulus) {
        return (base < 0) ? (modulus - (Math.abs(base) % modulus)) % modulus : base % modulus;
    }

    public static Pose2d relativeToReverse(Pose2d thisPose, Pose2d otherPose) {
        // Performs the reverse of Pose2d's relativeTo method
        // Pose2d.relativeTo applies a negative translation then negative rotation. 
        // Effectively this walks backwards in the transform tree.
        // This method walks forward in the transform tree.
        // It applies a positive rotation into the new frame and then a translation in that frame
        Translation2d translation = (
            thisPose.getTranslation()
            .rotateBy(otherPose.getRotation())
            .plus(otherPose.getTranslation()));

        Rotation2d rotation = thisPose.getRotation().plus(otherPose.getRotation());
        
        return new Pose2d(translation, rotation);
    }

}