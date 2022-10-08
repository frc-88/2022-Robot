package frc.robot.util.coprocessor.tunnel;

public enum PacketType {
    NORMAL(0),
    HANDSHAKE(1),
    CONFIRMING(2);
    
    public int type;

    private PacketType(int type) {
        this.type = type;
    }

    public static PacketType getType(int type) {
        for (PacketType item : PacketType.values()) {
            if (type == item.type) {
                return item;
            }
        }
        return NORMAL;
    }

    public int getValue() {
        return type;
    }
}
