package frc.robot.util.tunnel;

public interface TunnelInterface {
    public void packetCallback(TunnelClient tunnel, PacketResult result);
    public void update();
}
