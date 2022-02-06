package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.climber.ClimberArm;

public class Climber extends SubsystemBase {
    
    private final DigitalInput coastButton;

    private final ClimberArm outerLeftArm;
    private final ClimberArm outerRightArm;
    private final ClimberArm innerLeftArm;
    private final ClimberArm innerRightArm;

    private List<ClimberArm> allArms;

    public Climber() {
        outerLeftArm = new ClimberArm("Outer Left", Constants.OUTER_LEFT_CLIMBER_PIVOT_ID, Constants.OUTER_LEFT_CLIMBER_TELESCOPE_ID);
        outerRightArm = new ClimberArm("Outer Right", Constants.OUTER_RIGHT_CLIMBER_PIVOT_ID, Constants.OUTER_RIGHT_CLIMBER_TELESCOPE_ID);
        innerLeftArm = new ClimberArm("Inner Left", Constants.INNER_LEFT_CLIMBER_PIVOT_ID, Constants.INNER_LEFT_CLIMBER_TELESCOPE_ID);
        innerRightArm = new ClimberArm("Inner Right", Constants.INNER_RIGHT_CLIMBER_PIVOT_ID, Constants.INNER_RIGHT_CLIMBER_TELESCOPE_ID);

        coastButton = new DigitalInput(0);

        allArms = Arrays.asList(new ClimberArm[]{outerLeftArm, outerRightArm, innerLeftArm, innerRightArm});
    }

    public void calibrate() {
        allArms.forEach(ClimberArm::calibrate);
    }

    public void resetCalibration() {
        allArms.forEach(ClimberArm::resetCalibration);
    }

    public boolean isCalibrated() {
        return allArms.stream().allMatch(ClimberArm::isCalibrated);
    }


    public void setInnerPercentOutput(double innerPivotOutput, double innerTelescopeOutput) {
        this.innerLeftArm.setPercentOutput(innerPivotOutput, innerTelescopeOutput);
        this.innerRightArm.setPercentOutput(innerPivotOutput, innerTelescopeOutput);
    }

    public void setOuterPercentOutput(double outerPivotOutput, double outerTelescopeOutput) {
        this.outerLeftArm.setPercentOutput(outerPivotOutput, outerTelescopeOutput);
        this.outerRightArm.setPercentOutput(outerPivotOutput, outerTelescopeOutput);
    }

    public void setInnerMotionMagic(double pivotAngle, double telescopeHeight) {
        this.innerLeftArm.setMotionMagic(pivotAngle, telescopeHeight);
        this.innerRightArm.setMotionMagic(pivotAngle, telescopeHeight);
    }

    public void setOuterMotionMagic(double pivotAngle, double telescopeHeight) {
        this.outerLeftArm.setMotionMagic(pivotAngle, telescopeHeight);
        this.outerRightArm.setMotionMagic(pivotAngle, telescopeHeight);
    }


    public double getAverageInnerPivotAngle() {
        return (innerLeftArm.getPivotAngle() + innerRightArm.getPivotAngle()) / 2.;
    }

    public double getAverageInnerTelescopeHeight() {
        return (innerLeftArm.getTelescopeHeight() + innerRightArm.getTelescopeHeight()) / 2.;
    }

    public double getAverageOuterPivotAngle() {
        return (outerLeftArm.getPivotAngle() + outerRightArm.getPivotAngle()) / 2.;
    }

    public double getAverageOuterTelescopeHeight() {
        return (outerLeftArm.getTelescopeHeight() + outerRightArm.getTelescopeHeight()) / 2.;
    }


    @Override
    public void periodic() {
        allArms.forEach(ClimberArm::publishData);

        SmartDashboard.putBoolean("All Climbers Calibrated", isCalibrated());

        if (!coastButton.get()) {
            allArms.forEach(ClimberArm::coast);
        }
        else {
            allArms.forEach(ClimberArm::brake);
        }
        SmartDashboard.putBoolean("Coast Button", !coastButton.get());
    }

    @Override
    public void simulationPeriodic() {
        allArms.forEach(ClimberArm::simulate);
    }
}
