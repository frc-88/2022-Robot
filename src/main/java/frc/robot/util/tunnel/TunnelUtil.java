package frc.robot.util.tunnel;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class TunnelUtil {
    
    private static String toASCII(byte value) {
        int length = 4;
        StringBuilder builder = new StringBuilder(length);
        for (int i = length - 1; i >= 0; i--) {
            builder.append((char) ((value >> (8 * i)) & 0xFF));
        }
        return builder.toString();
    }
    
    public static String formatChar(byte c)
    {
        // c &= 0xff;
        if (c == 92) return "\\\\";
        else if (c == 9) return "\\t";
        else if (c == 10) return "\\n";
        else if (c == 13) return "\\r";
        else if (c == 11 || c == 12 || c <= 9 || (14 <= c && c <= 31) || 127 <= c)
        {
            return String.format("\\x%02x", c).toString();
        }
        else {
            return toASCII(c);
            // return String.valueOf(c) + " ";
        }
    }
    private static int toDigit(char hexChar) {
        int digit = Character.digit(hexChar, 16);
        if(digit == -1) {
            throw new IllegalArgumentException(
              "Invalid Hexadecimal Character: "+ hexChar);
        }
        return digit;
    }
    public static byte hexToByte(String hexString) {
        int firstDigit = toDigit(hexString.charAt(0));
        int secondDigit = toDigit(hexString.charAt(1));
        return (byte) ((firstDigit << 4) + secondDigit);
    }

    public static String packetToString(byte[] buffer)
    {
        return packetToString(buffer, buffer.length);
    }

    public static String packetToString(byte[] buffer, int length)
    {
        String s = "";
        for (int index = 0; index < length; index++) {
            s += formatChar(buffer[index]);
        }
        return s;
    }

    public static int toInt(byte[] buffer) {
        ByteBuffer wrapped = ByteBuffer.wrap(buffer);
        wrapped.order(ByteOrder.BIG_ENDIAN);
        return wrapped.getInt();
    }
    public static short toShort(byte[] buffer) {
        ByteBuffer wrapped = ByteBuffer.wrap(buffer);
        wrapped.order(ByteOrder.BIG_ENDIAN);
        return wrapped.getShort();
    }
    public static double toDouble(byte[] buffer) {
        ByteBuffer wrapped = ByteBuffer.wrap(buffer);
        wrapped.order(ByteOrder.LITTLE_ENDIAN);
        return wrapped.getDouble();
    }
    public static byte[] toUInt16Bytes(short number)
    {
        ByteBuffer dbuf = ByteBuffer.allocate(2);
        dbuf.order(ByteOrder.BIG_ENDIAN);
        dbuf.putShort(number);
        return dbuf.array();
    }
    public static byte[] toInt32Bytes(int number)
    {
        ByteBuffer dbuf = ByteBuffer.allocate(4);
        dbuf.order(ByteOrder.BIG_ENDIAN);
        dbuf.putInt(number);
        return dbuf.array();
    }
    public static byte[] toFloatBytes(double number)
    {
        ByteBuffer dbuf = ByteBuffer.allocate(8);
        dbuf.order(ByteOrder.LITTLE_ENDIAN);
        dbuf.putDouble(number);
        return dbuf.array();
    }
    public static int copyArray(byte[] main_array, int start_index, byte[] secondary_array) {
        for (int index = 0; index < secondary_array.length; index++) {
            main_array[start_index++] = secondary_array[index];
            if (start_index >= main_array.length) {
                break;
            }
        }
        return start_index;
    }
}
