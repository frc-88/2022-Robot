package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.climber.ClimberArm;

public class Climber extends SubsystemBase {
    
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

        allArms = Arrays.asList(new ClimberArm[]{outerLeftArm, outerRightArm, innerLeftArm, innerRightArm});
    }

    public void calibrate() {
        allArms.forEach(ClimberArm::calibrate);
    }

    public boolean isCalibrated() {
        return allArms.stream().allMatch(ClimberArm::isCalibrated);
    }

    @Override
    public void periodic() {
        allArms.forEach(ClimberArm::publishData);
    }
}
