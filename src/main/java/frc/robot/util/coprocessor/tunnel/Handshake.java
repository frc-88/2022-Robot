// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.coprocessor.tunnel;

/** Add your docs here. */
public class Handshake {
    private String category = "";
    private int packet_num = 0;
    private long write_interval = 0;
    private long timeout = 0;

    private long initial_write_time = 0;
    private long prev_write_time = 0;
    private int attempt_counter = 0;
    private byte[] packet;

    public Handshake(String category, int packet_num, long write_interval, long timeout) {
        this.category = category;
        this.packet_num = packet_num;
        this.write_interval = write_interval;
        this.timeout = timeout;
    }

    public String getCategory() {
        return category;
    }
    public int getPacketNum() {
        return packet_num;
    }
    public void setPacket(byte[] packet){
        this.packet = packet;
    }
    public byte[] getPacket() {
        return packet;
    }
    public void setInitialWrite(long current_time) {
        initial_write_time = current_time;
        prev_write_time = current_time;
    }
    public boolean shouldWriteAgain(long current_time)
    {
        if (write_interval <= 0) {
            return false;
        }
        long dt = current_time - prev_write_time;
        if (dt > write_interval) {
            prev_write_time = current_time;
            attempt_counter++;
            return true;
        }
        else {
            return false;
        }
    }
    public boolean didFail(long current_time)
    {
        if (timeout > 0.0) {
            return current_time - initial_write_time > timeout;
        }
        else {
            return false;
        }
    }
    public boolean isEqual(String category, int packet_num) {
        return category.equals(category) && this.packet_num == packet_num;
    }
}
