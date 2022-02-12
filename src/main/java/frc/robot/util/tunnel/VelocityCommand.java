package frc.robot.util.tunnel;

public class VelocityCommand {
    public double vx = 0.0;  // x velocity (robot centric) in meters per second
    public double vy = 0.0;  // y velocity (robot centric) in meters per second
    public double vt = 0.0;  // rotational velocity (robot centric) in radians per second

    public VelocityCommand() {

    }

    public VelocityCommand(double vx, double vy, double vt)
    {
        this.vx = vx;
        this.vy = vy;
        this.vt = vt;
    }
}
