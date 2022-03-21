package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.climber.ClimberArm;

public class Climber extends SubsystemBase {
    
    private final BooleanSupplier coastButton;

    public final ClimberArm outerArm;
    public final ClimberArm innerArm;

    private List<ClimberArm> allArms;

    public Climber(BooleanSupplier coastButton) {
        this.coastButton = coastButton;

        outerArm = new ClimberArm("Outer Left", Constants.OUTER_CLIMBER_PIVOT_ID, Constants.OUTER_CLIMBER_TELESCOPE_ID, false, false);
        innerArm = new ClimberArm("Inner Right", Constants.INNER_CLIMBER_PIVOT_ID, Constants.INNER_CLIMBER_TELESCOPE_ID, true, false);

        allArms = Arrays.asList(new ClimberArm[]{outerArm, innerArm});
    }

    public void calibrate() {
        if (!innerArm.isCalibrated()) {
            innerArm.calibrate();
        } else if (!outerArm.isCalibrated()) {
            outerArm.calibrate();
        }
    }

    public void resetCalibration() {
        allArms.forEach(ClimberArm::resetCalibration);
    }

    public boolean isCalibrated() {
        return allArms.stream().allMatch(ClimberArm::isCalibrated);
    }


    public void setInnerPercentOutput(double innerPivotOutput, double innerTelescopeOutput) {
        this.innerArm.setPercentOutput(innerPivotOutput, innerTelescopeOutput);
    }

    public void setOuterPercentOutput(double outerPivotOutput, double outerTelescopeOutput) {
        this.outerArm.setPercentOutput(outerPivotOutput, outerTelescopeOutput);
    }

    public void setInnerMotionMagic(double pivotAngle, double telescopeHeight, double pivotSpeed, double telescopeSpeed) {
        this.innerArm.setMotionMagic(pivotAngle, telescopeHeight, pivotSpeed, telescopeSpeed);
    }

    public void setOuterMotionMagic(double pivotAngle, double telescopeHeight, double pivotSpeed, double telescopeSpeed) {
        this.outerArm.setMotionMagic(pivotAngle, telescopeHeight, pivotSpeed, telescopeSpeed);
    }

    public void setInnerMotionMagic(double pivotAngle, double telescopeHeight) {
        this.innerArm.setMotionMagic(pivotAngle, telescopeHeight);
    }

    public void setOuterMotionMagic(double pivotAngle, double telescopeHeight) {
        this.outerArm.setMotionMagic(pivotAngle, telescopeHeight);
    }


    public double getAverageInnerPivotAngle() {
        return (innerArm.getPivotAngle() + innerArm.getPivotAngle()) / 2.;
    }

    public double getAverageInnerTelescopeHeight() {
        return (innerArm.getTelescopeHeight() + innerArm.getTelescopeHeight()) / 2.;
    }

    public double getAverageOuterPivotAngle() {
        return (outerArm.getPivotAngle() + outerArm.getPivotAngle()) / 2.;
    }

    public double getAverageOuterTelescopeHeight() {
        return (outerArm.getTelescopeHeight() + outerArm.getTelescopeHeight()) / 2.;
    }


    @Override
    public void periodic() {
        allArms.forEach(ClimberArm::publishData);

        SmartDashboard.putBoolean("All Climbers Calibrated", isCalibrated());

        if (coastButton.getAsBoolean()) {
            allArms.forEach(ClimberArm::coast);
        }
        else {
            allArms.forEach(ClimberArm::brake);
        }
    }

    @Override
    public void simulationPeriodic() {
        allArms.forEach(ClimberArm::simulate);
    }
}
