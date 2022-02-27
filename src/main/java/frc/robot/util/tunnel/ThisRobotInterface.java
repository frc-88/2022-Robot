package frc.robot.util.tunnel;

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

public class ThisRobotInterface extends ROSInterface {
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

    private ClimberArm outerLeftArm;
    private ClimberArm outerRightArm;
    private ClimberArm innerLeftArm;
    private ClimberArm innerRightArm;

    private Intake intake;
    private Turret turret;
    private Sensors sensors;

    public ThisRobotInterface(ChassisInterface chassis,
                              ClimberArm outerLeftArm, ClimberArm outerRightArm, ClimberArm innerLeftArm, ClimberArm innerRightArm,
                              Intake intake,
                              Turret turret,
                              Sensors sensors) {
        super(chassis);
        this.outerLeftArm = outerLeftArm;
        this.outerRightArm = outerRightArm;
        this.innerLeftArm = innerLeftArm;
        this.innerRightArm = innerRightArm;
        this.intake = intake;
        this.turret = turret;
        this.sensors = sensors;
    }

    // @Override
    // public void packetCallback(TunnelClient tunnel, PacketResult result) {
    //     super.packetCallback(tunnel, result);
    // }

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

    @Override
    public void updateSlow() {
        super.updateSlow();
        Vector2D outerLeftArmVector = outerLeftArm.getPositionVector();
        Vector2D outerRightArmVector = outerRightArm.getPositionVector();
        Vector2D innerLeftArmVector = innerLeftArm.getPositionVector();
        Vector2D innerRightArmVector = innerRightArm.getPositionVector();

        // outerLeftArmVector
        TunnelServer.writePacket("joint",
            left_outer_climber_joint,
            convertClimberPivotAngle(outerLeftArmVector.getAngle())
        );

        TunnelServer.writePacket("joint",
            left_outer_climber_hook_joint,
            convertClimberTelescopeHeight(outerLeftArmVector.getMagnitude())
        );

        // innerLeftArmVector
        TunnelServer.writePacket("joint",
            left_inner_climber_joint,
            convertClimberPivotAngle(innerLeftArmVector.getAngle())
        );

        TunnelServer.writePacket("joint",
            left_inner_climber_hook_joint,
            convertClimberTelescopeHeight(innerLeftArmVector.getMagnitude())
        );

        // outerRightArmVector
        TunnelServer.writePacket("joint",
            right_outer_climber_joint,
            convertClimberPivotAngle(outerRightArmVector.getAngle())
        );

        TunnelServer.writePacket("joint",
            right_outer_climber_hook_joint,
            convertClimberTelescopeHeight(outerRightArmVector.getMagnitude())
        );

        // innerRightArmVector
        TunnelServer.writePacket("joint",
            right_inner_climber_joint,
            convertClimberPivotAngle(innerRightArmVector.getAngle())
        );

        TunnelServer.writePacket("joint",
            right_inner_climber_hook_joint,
            convertClimberTelescopeHeight(innerRightArmVector.getMagnitude())
        );

        // intake
        TunnelServer.writePacket("joint",
            intake_joint,
            convertIntakeAngle(intake.getArmPosition())
        );

        // turret
        TunnelServer.writePacket("joint",
            turret_joint,
            convertTurretAngle(turret.getFacing())
        );

        // camera
        TunnelServer.writePacket("joint",
            camera_joint,
            convertCameraTiltAngle(sensors.getCameraTilterAngle())
        );
    }
}
