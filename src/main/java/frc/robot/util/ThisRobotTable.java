package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Turret;
import frc.robot.util.Vector2D;
import frc.robot.util.WrappedAngle;
import frc.robot.util.climber.ClimberArm;
import frc.robot.util.coprocessortable.ChassisInterface;
import frc.robot.util.coprocessortable.CoprocessorTable;

public class ThisRobotTable extends CoprocessorTable {
    final int left_outer_climber_joint = 0;
    final int left_inner_climber_joint = 1;
    final int right_outer_climber_joint = 2;
    final int right_inner_climber_joint = 3;
    final int left_outer_climber_hook_joint = 4;
    final int left_inner_climber_hook_joint = 5;
    final int right_outer_climber_hook_joint = 6;
    final int right_inner_climber_hook_joint = 7;
    final int intake_joint = 8;
    final int turret_joint = 9;
    final int camera_joint = 10;

    private ClimberArm outerArm;
    private ClimberArm innerArm;

    private Intake intake;
    private Turret turret;
    private Sensors sensors;

    public ThisRobotTable(
        ChassisInterface chassis, String address, int port, double updateInterval,
            ClimberArm outerArm, ClimberArm innerArm,
            Intake intake,
            Turret turret,
            Sensors sensors) {
        super(chassis, address, port, updateInterval);

        this.outerArm = outerArm;
        this.innerArm = innerArm;
        this.intake = intake;
        this.turret = turret;
        this.sensors = sensors;
    }

    // @Override
    // public void update() {
    //     super.update();
    // }

    public void updateSlow() {
        if (!isConected()) {
            return;
        }
        
        Vector2D outerLeftArmVector = outerArm.getPositionVector();
        Vector2D outerRightArmVector = outerArm.getPositionVector();
        Vector2D innerLeftArmVector = innerArm.getPositionVector();
        Vector2D innerRightArmVector = innerArm.getPositionVector();

        // outerLeftArmVector
        setJointPosition(
            left_outer_climber_joint,
            convertClimberPivotAngle(outerLeftArmVector.getAngle())
        );

        setJointPosition(
            left_outer_climber_hook_joint,
            convertClimberTelescopeHeight(outerLeftArmVector.getMagnitude())
        );

        // innerLeftArmVector
        setJointPosition(
            left_inner_climber_joint,
            convertClimberPivotAngle(innerLeftArmVector.getAngle())
        );

        setJointPosition(
            left_inner_climber_hook_joint,
            convertClimberTelescopeHeight(innerLeftArmVector.getMagnitude())
        );

        // outerRightArmVector
        setJointPosition(
            right_outer_climber_joint,
            convertClimberPivotAngle(outerRightArmVector.getAngle())
        );

        setJointPosition(
            right_outer_climber_hook_joint,
            convertClimberTelescopeHeight(outerRightArmVector.getMagnitude())
        );

        // innerRightArmVector
        setJointPosition(
            right_inner_climber_joint,
            convertClimberPivotAngle(innerRightArmVector.getAngle())
        );

        setJointPosition(
            right_inner_climber_hook_joint,
            convertClimberTelescopeHeight(innerRightArmVector.getMagnitude())
        );

        // intake
        setJointPosition(
            intake_joint,
            convertIntakeAngle(intake.getArmPosition())
        );

        // turret
        setJointPosition(
            turret_joint,
            convertTurretAngle(turret.getFacing())
        );

        // camera
        setJointPosition(
            camera_joint,
            convertCameraTiltAngle(sensors.getCameraTilterAngle())
        );
    }


    private double convertClimberPivotAngle(WrappedAngle pivotAngle) {
        return Math.toRadians(pivotAngle.asDouble());
    }

    private double convertClimberTelescopeHeight(double telescopeHeight) {
        return Units.inchesToMeters(telescopeHeight - ClimberArm.TELESCOPE_MIN_HEIGHT);
    }

    private static final double ROS_INTAKE_ARM_DEPLOYED = 0.0;
    private static final double ROS_INTAKE_ARM_STOWED = -93.0;
    private double convertIntakeAngle(double intakeAngle) {
        return Math.toRadians(
            (Intake.ARM_DEPLOYED - Intake.ARM_STOWED) / 
            (ROS_INTAKE_ARM_DEPLOYED - ROS_INTAKE_ARM_STOWED) * 
            (intakeAngle - Intake.ARM_STOWED)
             + ROS_INTAKE_ARM_STOWED
        );
    }

    private double convertTurretAngle(double turretAngle) {
        return Math.toRadians(turretAngle);
    }

    private double convertCameraTiltAngle(Rotation2d cameraAngle) {
        return cameraAngle.getRadians();
    }

    public String getGameObjectName() {
        Alliance team_color = DriverStation.getAlliance();
        String object_name = "";
        if (team_color == Alliance.Red) {
            object_name = "cargo_red";
        }
        else if (team_color == Alliance.Blue) {
            object_name = "cargo_blue";
        }
        return object_name;
    }
}
