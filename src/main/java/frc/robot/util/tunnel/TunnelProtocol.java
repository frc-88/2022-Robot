package frc.robot.util.tunnel;

import edu.wpi.first.wpilibj.RobotController;

import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;

public class TunnelProtocol {
    public static final char PACKET_START_0 = 0x12;
    public static final char PACKET_START_1 = 0x13;
    public static final char PACKET_STOP = '\n';
    public static final char PACKET_SEP = '\t';
    public static final int MAX_PACKET_LEN = 1024;
    public static final int MIN_PACKET_LEN = 13;
    public static final int MAX_SEGMENT_LEN = 64;
    public static final int CHECKSUM_START_INDEX = 4;
    public static final int LENGTH_START_INDEX = 2;
    public static final int LENGTH_BYTE_LENGTH = 2;

    public static final int NULL_ERROR = -1;
    public static final int NO_ERROR = 0;
    public static final int PACKET_0_ERROR = 1;
    public static final int PACKET_1_ERROR = 2;
    public static final int PACKET_TOO_SHORT_ERROR = 3;
    public static final int CHECKSUMS_DONT_MATCH_ERROR = 4;
    public static final int PACKET_COUNT_NOT_FOUND_ERROR = 5;
    public static final int PACKET_COUNT_NOT_SYNCED_ERROR = 6;
    public static final int PACKET_CATEGORY_ERROR = 7;
    public static final int INVALID_FORMAT_ERROR = 8;
    public static final int PACKET_STOP_ERROR = 9;
    public static final int SEGMENT_TOO_LONG_ERROR = 10;
    public static final int PACKET_TIMEOUT_ERROR = 11;

    private int read_packet_num = -1;
    private int write_packet_num = 0;
    
    private byte[] write_buffer = new byte[MAX_PACKET_LEN];
    private int write_buffer_index = 0;
    private int read_buffer_index = 0;

    private byte[] current_segment;
    
    private ArrayList<PacketResult> resultQueue = new ArrayList<PacketResult>();

    public TunnelProtocol()
    {
        
    }

    public byte[] makePacket(String category, Object... args)
    {
        applyPacketHeader(category);
        for (Object object : args) {
            if (object instanceof Integer) {
                write_buffer_index = TunnelUtil.copyArray(write_buffer, write_buffer_index, TunnelUtil.toInt32Bytes((int)object));
            }
            else if (object instanceof Boolean) {
                write_buffer_index = TunnelUtil.copyArray(write_buffer, write_buffer_index, TunnelUtil.toInt32Bytes((boolean)object ? 1 : 0));
            }
            else if (object instanceof Double) {
                write_buffer_index = TunnelUtil.copyArray(write_buffer, write_buffer_index, TunnelUtil.toFloatBytes((double)object));
            }
            else if (object instanceof String) {
                short length = (short)((String)object).length();

                write_buffer_index = TunnelUtil.copyArray(write_buffer, write_buffer_index, TunnelUtil.toUInt16Bytes(length));
                write_buffer_index = TunnelUtil.copyArray(write_buffer, write_buffer_index, ((String)object).getBytes());
            }
            else {
                System.out.println(String.format(
                    "Encountered invalid type while making packet for category '%s'. Object '%s' is of type '%s'",
                    category,
                    object,
                    object.getClass().getName())
                );
                return new byte[0];
            }
        }
        boolean success = applyPacketFooter();
        if (success) {
            return Arrays.copyOfRange(write_buffer, 0, write_buffer_index);
        }
        else {
            return new byte[0];
        }
    }

    private void applyPacketHeader(String category)
    {
        write_buffer_index = 0;
        write_buffer[write_buffer_index++] = PACKET_START_0;
        write_buffer[write_buffer_index++] = PACKET_START_1;
        write_buffer_index += LENGTH_BYTE_LENGTH;  // two bytes for packet length
        byte[] packet_num_bytes = TunnelUtil.toInt32Bytes(write_packet_num);
        write_packet_num++;
        write_buffer_index = TunnelUtil.copyArray(write_buffer, write_buffer_index, packet_num_bytes);
        byte[] category_bytes = category.getBytes();
        write_buffer_index = TunnelUtil.copyArray(write_buffer, write_buffer_index, category_bytes);
        write_buffer[write_buffer_index++] = PACKET_SEP;
    }

    private boolean applyPacketFooter()
    {
        if (CHECKSUM_START_INDEX > write_buffer_index) {
            System.out.println(String.format(
                "Invalid packet! Packet length is greater than checksum start index: %s. write_buffer_index = %d",
                TunnelUtil.packetToString(write_buffer),
                write_buffer_index
            ));
            return false;
        }
        byte calc_checksum = calculateChecksum(Arrays.copyOfRange(write_buffer, CHECKSUM_START_INDEX, write_buffer_index));
        
        write_buffer_index = TunnelUtil.copyArray(write_buffer, write_buffer_index,
            String.format("%02x", calc_checksum).getBytes()
        );
        TunnelUtil.copyArray(write_buffer, LENGTH_START_INDEX, 
            TunnelUtil.toUInt16Bytes(
                (short)(write_buffer_index - (LENGTH_START_INDEX + LENGTH_BYTE_LENGTH))
            )
        );
        write_buffer[write_buffer_index++] = PACKET_STOP;

        return true;
    }

    public byte calculateChecksum(byte[] packet) {
        byte calc_checksum = 0;
        for (byte val : packet) {
            calc_checksum += val;
        }
        return calc_checksum;
    }

    public boolean isCodeError(int error_code) {
        switch (error_code) {
            case NO_ERROR:
            case PACKET_COUNT_NOT_SYNCED_ERROR:
            case NULL_ERROR:
                return false;
            default:
                return true;
        }
    }
    public int parseBuffer(byte[] buffer)
    {
        // returns where to move the start index to
        // TODO enforce max packet length
        int last_packet_index = 0;
        int index;
        for (index = 0; index < buffer.length; index++) {
            int packet_start = index;
            if ((char)buffer[index] != PACKET_START_0) {
                // System.out.println(buffer[index] + " is not PACKET_START_0");
                continue;
            }
            index++;
            if (index >= buffer.length) {
                index = buffer.length;
                continue;
            }
            if ((char)buffer[index] != PACKET_START_1) {
                // System.out.println(buffer[index] + " is not PACKET_START_1");
                continue;
            }
            index++;
            if (index >= buffer.length) {
                index = buffer.length;
                continue;
            }
            int length_start = index;
            index += 2;
            if (index >= buffer.length) {
                index = buffer.length;
                // System.out.println("Buffer length exceeded while searching for length start");
                continue;
            }
            
            short length = TunnelUtil.toShort(Arrays.copyOfRange(buffer, length_start, index));
            // System.out.println("Found packet length: " + length);

            index += length;
            if (index > buffer.length) {
                index = buffer.length;
                // System.out.println(String.format("Buffer length exceeded while searching for length stop. %d > %d", index, buffer.length));
                continue;
            }
            if (length > MAX_PACKET_LEN) {
                // System.out.println(String.format("Packet length exceeds max allowable packet size (%d > %d). Skipping", length, MAX_PACKET_LEN));
                continue;
            }
            if ((char)buffer[index] != PACKET_STOP) {
                // System.out.println("Buffer does not end with PACKET_STOP");
                continue;
            }
            // do not modify index from this point onward as the for loop increments index
            last_packet_index = index + 1;
            byte[] packet = Arrays.copyOfRange(buffer, packet_start, index + 1);
            // System.out.println("Found a packet: " + TunnelUtil.packetToString(packet));
            PacketResult result = parsePacket(packet);
            resultQueue.add(result);
        }

        return last_packet_index;
    }

    private long getTime()
    {
        return RobotController.getFPGATime();
    }
    
    public PacketResult parsePacket(byte[] buffer)
    {
        this.read_buffer_index = 0;
        long recv_time = getTime();
        if (buffer.length < MIN_PACKET_LEN) {
            System.out.println(String.format("Packet is not the minimum length (%s): %s", MIN_PACKET_LEN, TunnelUtil.packetToString(buffer)));
            return new PacketResult(PACKET_TOO_SHORT_ERROR, recv_time);
        }

        if (buffer[0] != PACKET_START_0) {
            System.out.println(String.format("Packet does not start with PACKET_START_0: %s", TunnelUtil.packetToString(buffer)));
            read_packet_num++;
            return new PacketResult(PACKET_0_ERROR, recv_time);
        }
        this.read_buffer_index++;
        if (buffer[1] != PACKET_START_1) {
            System.out.println(String.format("Packet does not start with PACKET_START_1: %s", TunnelUtil.packetToString(buffer)));
            read_packet_num++;
            return new PacketResult(PACKET_1_ERROR, recv_time);
        }
        this.read_buffer_index++;
        if (buffer[buffer.length - 1] != PACKET_STOP) {
            System.out.println(String.format("Packet does not start with PACKET_STOP: %s", TunnelUtil.packetToString(buffer)));
            read_packet_num++;
            return new PacketResult(PACKET_STOP_ERROR, recv_time);
        }
        int checksum_start = buffer.length - 3;
        byte calc_checksum = calculateChecksum(Arrays.copyOfRange(buffer, LENGTH_START_INDEX + LENGTH_BYTE_LENGTH, checksum_start));

        byte recv_checksum;
        try {
            recv_checksum = TunnelUtil.hexToByte(new String(Arrays.copyOfRange(buffer, checksum_start, buffer.length - 1)));
        }
        catch (IllegalArgumentException e) {
            e.printStackTrace();
            System.out.println(String.format("Checksum failed! Illegal character encountered. %s", TunnelUtil.packetToString(buffer)));
            read_packet_num++;
            return new PacketResult(CHECKSUMS_DONT_MATCH_ERROR, recv_time);
        } 
        if (calc_checksum != recv_checksum) {
            System.out.println(String.format("Checksum failed! recv %02x != calc %02x. %s", recv_checksum, calc_checksum, TunnelUtil.packetToString(buffer)));
            read_packet_num++;
            return new PacketResult(CHECKSUMS_DONT_MATCH_ERROR, recv_time);
        }
        this.read_buffer_index += 2;

        if (!getNextSegment(buffer, 4)) {
            System.out.println(String.format("Failed to find packet number segment! %s", TunnelUtil.packetToString(buffer)));
            read_packet_num++;
            return new PacketResult(PACKET_COUNT_NOT_FOUND_ERROR, recv_time);
        }
        int recv_packet_num = TunnelUtil.toInt(this.current_segment);
        if (this.read_packet_num == -1) {
            this.read_packet_num = recv_packet_num;
        }

        PacketResult result = new PacketResult();

        if (recv_packet_num != this.read_packet_num) {
            System.out.println(String.format("Received packet num doesn't match local count. recv %d != local %d", recv_packet_num, this.read_packet_num));
            this.read_packet_num = recv_packet_num;
            result.setErrorCode(PACKET_COUNT_NOT_SYNCED_ERROR);
        }

        if (!getNextSegment(buffer)) {
            System.out.println(String.format(
                "Failed to find category segment %s! %s",
                TunnelUtil.packetToString(this.current_segment),
                TunnelUtil.packetToString(buffer))
            );
            read_packet_num++;
            return new PacketResult(PACKET_CATEGORY_ERROR, recv_time);
        }

        String category = new String(this.current_segment, StandardCharsets.UTF_8);
        if (category.length() == 0) {
            System.out.println(String.format(
                "Category segment is empty: %s, %s",
                TunnelUtil.packetToString(this.current_segment),
                TunnelUtil.packetToString(buffer))
            );
            read_packet_num++;
            return new PacketResult(PACKET_CATEGORY_ERROR, recv_time);
        }
        result.setCategory(category);

        // read_buffer_index is currently the next index after category separator (\t)
        result.setStart(this.read_buffer_index);
        result.setStop(checksum_start + 1);

        result.setBuffer(buffer);
        result.setErrorCode(NO_ERROR);
        read_packet_num++;
        
        return result;
    }

    private boolean getNextSegment(byte[] buffer, int length) {
        if (this.read_buffer_index >= buffer.length) {
            return false;
        }
        if (length == -1)
        {
            // assume first 2 bytes contain the length
            byte[] len_bytes = Arrays.copyOfRange(buffer, this.read_buffer_index, this.read_buffer_index + 2);
            length = TunnelUtil.toInt(len_bytes);
            this.read_buffer_index += 2;
            if (length >= buffer.length) {
                System.out.println(String.format("Parsed length %s exceeds buffer length! %s", length, buffer.length));
                return false;
            }
        }
        this.current_segment = Arrays.copyOfRange(buffer, this.read_buffer_index, this.read_buffer_index + length);
        this.read_buffer_index += length;
        return true;
    }

    private boolean getNextSegment(byte[] buffer) {  // search for next PACKET_SEP
        if (read_buffer_index >= buffer.length) {
            return false;
        }
        int sep_index;
        for (sep_index = read_buffer_index; sep_index < buffer.length; sep_index++) {
            if (buffer[sep_index] == PACKET_SEP) {
                break;
            }
        }
        if (sep_index >= buffer.length) {
            this.current_segment = Arrays.copyOfRange(buffer, this.read_buffer_index, buffer.length);
            this.read_buffer_index = buffer.length;
        }
        else {
            this.current_segment = Arrays.copyOfRange(buffer, this.read_buffer_index, sep_index);
            this.read_buffer_index = sep_index + 1;
        }
        return true;
    }

    public PacketResult popResult() {
        if (resultQueue.size() <= 0) {
            return new PacketResult(NULL_ERROR, 0);
        }
        PacketResult result = resultQueue.get(0);
        resultQueue.remove(0);
        return result;
    }

    public static void main(String[] args) {
        TunnelProtocol protocol = new TunnelProtocol();
        
        byte[] packet = protocol.makePacket("ping", 4.0);
        String packet_str = TunnelUtil.packetToString(packet);
        System.out.println(packet_str);
        protocol.parseBuffer(packet);
        PacketResult result = protocol.popResult();
        System.out.println("Error code: " + result.getErrorCode());
        System.out.println("Category: " + result.getCategory());
        System.out.println(String.format("Data: %f", result.getDouble()));

        packet = protocol.makePacket("cmd", 5.3, 2.1, -6.6);
        packet_str = TunnelUtil.packetToString(packet);
        System.out.println(packet_str);
        protocol.parseBuffer(packet);
        result = protocol.popResult();
        System.out.println("Error code: " + result.getErrorCode());
        System.out.println("Category: " + result.getCategory());
        System.out.println(String.format("1: %f", result.getDouble()));
        System.out.println(String.format("2: %f", result.getDouble()));
        System.out.println(String.format("3: %f", result.getDouble()));

        // packet_str = TunnelUtil.packetToString(protocol.makePacket("something", 50.0, 10, "else"));
        // System.out.println(packet_str);
    }
}
