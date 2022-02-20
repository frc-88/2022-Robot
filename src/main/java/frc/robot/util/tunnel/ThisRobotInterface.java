package frc.robot.util.tunnel;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Turret;
import frc.robot.util.Vector2D;
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
    // private CameraTilter cameraTilter;  // TODO add camera tilter when that's ready

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


    @Override
    public void update() {
        super.update();
        Vector2D outerLeftArmVector = outerLeftArm.getPositionVector();
        Vector2D outerRightArmVector = outerRightArm.getPositionVector();
        Vector2D innerLeftArmVector = innerLeftArm.getPositionVector();
        Vector2D innerRightArmVector = innerRightArm.getPositionVector();

        // outerLeftArmVector
        TunnelServer.writePacket("joint",
            left_outer_climber_joint,
            Math.toRadians(outerLeftArmVector.getAngle().asDouble())
        );

        TunnelServer.writePacket("joint",
            left_outer_climber_hook_joint,
            Units.inchesToMeters(outerLeftArmVector.getMagnitude())
        );

        // innerLeftArmVector
        TunnelServer.writePacket("joint",
            left_inner_climber_joint,
            Math.toRadians(innerLeftArmVector.getAngle().asDouble())
        );

        TunnelServer.writePacket("joint",
            left_inner_climber_hook_joint,
            Units.inchesToMeters(innerLeftArmVector.getMagnitude())
        );

        // outerRightArmVector
        TunnelServer.writePacket("joint",
            right_outer_climber_joint,
            Math.toRadians(outerRightArmVector.getAngle().asDouble())
        );

        TunnelServer.writePacket("joint",
            right_outer_climber_hook_joint,
            Units.inchesToMeters(outerRightArmVector.getMagnitude())
        );

        // innerRightArmVector
        TunnelServer.writePacket("joint",
            right_inner_climber_joint,
            Math.toRadians(innerRightArmVector.getAngle().asDouble())
        );

        TunnelServer.writePacket("joint",
            right_inner_climber_hook_joint,
            Units.inchesToMeters(innerRightArmVector.getMagnitude())
        );

        // intake
        TunnelServer.writePacket("joint",
            intake_joint,
            Math.toRadians(intake.getArmPosition())
        );

        // turret
        TunnelServer.writePacket("joint",
            turret_joint,
            Math.toRadians(turret.getPosition())
        );

        // camera
        TunnelServer.writePacket("joint",
            camera_joint,
            sensors.getCameraTilterAngle().getRadians()
        );
    }

}
