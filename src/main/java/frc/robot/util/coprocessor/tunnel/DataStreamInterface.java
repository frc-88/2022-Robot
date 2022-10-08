package frc.robot.util.coprocessor.tunnel;

public interface DataStreamInterface {
    public void writePacket(String category, Object... objects);
}
