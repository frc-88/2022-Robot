package frc.robot.util.coprocessor.tunnel;

import java.util.Arrays;

import edu.wpi.first.math.Pair;

public class PacketResult {
    private String category = "";
    private int error_code = 0;
    private long recv_time = 0;
    private byte[] buffer;
    private int start_index;
    private int stop_index;
    private int current_index;
    private PacketType packet_type = PacketType.NORMAL;
    private int packet_num = 0;

    public PacketResult()
    {
        
    }

    public PacketResult(int error_code, long recv_time)
    {
        this.error_code = error_code;
        this.recv_time = recv_time;
    }

    public void setCategory(String category) {
        this.category = category;
    }
    public String getCategory() {
        return this.category;
    }

    public void setPacketType(PacketType packet_type) {
        this.packet_type = packet_type;
    }
    public PacketType getPacketType() {
        return this.packet_type;
    }

    public void setPacketNum(int packet_num) {
        this.packet_num = packet_num;
    }
    public int getPacketNum() {
        return this.packet_num;
    }

    public void setErrorCode(int error_code) {
        if (error_code != 0) {
            return;
        }
        this.error_code = error_code;
    }
    public int getErrorCode() {
        return this.error_code;
    }

    public void setRecvTime(long recv_time) {
        this.recv_time = recv_time;
    }
    public long getRecvTime() {
        return this.recv_time;
    }

    public void setBuffer(byte[] buffer) {
        this.buffer = buffer;
    }

    public void setStart(int index) {
        this.start_index = index;
        this.current_index = this.start_index;
    }
    public void setStop(int index) {
        this.stop_index = index;
    }

    private boolean checkIndex() {
        if (this.current_index >= this.stop_index) {
            System.out.println(
                String.format("Index exceeds buffer limits. %d >= %d", this.current_index, this.stop_index)
            );
            return false;
        }
        return true;
    }

    public Pair<Integer, Boolean> getInt() {
        int next_index = this.current_index + 4;
        int result = TunnelUtil.toInt(Arrays.copyOfRange(buffer, this.current_index, next_index));
        this.current_index = next_index;
        return new Pair<Integer, Boolean>(result, checkIndex());
    }
    public Pair<Double, Boolean> getDouble() {
        int next_index = this.current_index + 8;
        double result = TunnelUtil.toDouble(Arrays.copyOfRange(buffer, this.current_index, next_index));
        this.current_index = next_index;
        return new Pair<Double, Boolean>(result, checkIndex());
    }
    public Pair<String, Boolean> getString() {
        int next_index = this.current_index + 2;
        short length = TunnelUtil.toShort(Arrays.copyOfRange(buffer, this.current_index, next_index));
        this.current_index = next_index;
        if (!checkIndex()) {
            return new Pair<String, Boolean>("", false);
        }
        Pair<String, Boolean> result = getString(length);
        return result;
    }
    public Pair<String, Boolean> getString(int length) {
        if (length > TunnelProtocol.MAX_SEGMENT_LEN) {
            System.out.println("String length exceeds max segment length: " + length);
            return new Pair<String, Boolean>("", false);
        }
        int next_index = this.current_index + length;
        String result = Arrays.toString(Arrays.copyOfRange(buffer, this.current_index, next_index));
        this.current_index = next_index;
        return new Pair<String, Boolean>(result, checkIndex());
    }
}
