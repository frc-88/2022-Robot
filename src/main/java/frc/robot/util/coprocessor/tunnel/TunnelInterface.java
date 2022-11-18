package frc.robot.util.coprocessor.tunnel;

public interface TunnelInterface {
    public void packetCallback(DataStreamInterface data_stream, PacketResult result);
    public void handshakeCallback(DataStreamInterface data_stream, Handshake handshake);
    public void update();
}
