package frc.robot.util.tunnel;

import java.util.Arrays;

public class PacketResult {
    private String category = "";
    private int error_code = 0;
    private long recv_time = 0;
    private byte[] buffer;
    private int start_index;
    private int stop_index;
    private int current_index;

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

    private void checkIndex() {
        if (this.current_index >= this.stop_index) {
            throw new RuntimeException(
                String.format("Index exceeds buffer limits. %d >= %d", this.current_index, this.stop_index));
        }
    }

    public int getInt() {
        int next_index = this.current_index + 4;
        int result = TunnelUtil.toInt(Arrays.copyOfRange(buffer, this.current_index, next_index));
        this.current_index = next_index;
        checkIndex();
        return result;
    }
    public double getDouble() {
        int next_index = this.current_index + 8;
        double result = TunnelUtil.toDouble(Arrays.copyOfRange(buffer, this.current_index, next_index));
        this.current_index = next_index;
        checkIndex();
        return result;
    }
    public String getString() {
        int next_index = this.current_index + 2;
        short length = TunnelUtil.toShort(Arrays.copyOfRange(buffer, this.current_index, next_index));
        this.current_index = next_index;
        return getString(length);
    }
    public String getString(int length) {
        int next_index = this.current_index + length;
        String result = Arrays.toString(Arrays.copyOfRange(buffer, this.current_index, next_index));
        this.current_index = next_index;
        checkIndex();
        return result;
    }
}
