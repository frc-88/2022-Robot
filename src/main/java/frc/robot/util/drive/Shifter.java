package frc.robot.util.drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Robot;

public class Shifter {

    public static class ShifterParameters {
        public PneumaticsModuleType pchType;
        public int pchId;
        public int outId;
        public int inId;

        public int cancoderId;
        public double minPosition;
        public double maxPosition;
        public double lowThreshold;
        public double highThreshold;

        public ShifterParameters(PneumaticsModuleType pchType, int pchId, int outId, int inId, int cancoderId, double minPosition, double maxPosition, double lowThreshold, double highThreshold) {
            this.pchType = pchType;
            this.pchId = pchId;
            this.outId = outId;
            this.inId = inId;
            this.cancoderId = cancoderId;
            this.minPosition = minPosition;
            this.maxPosition = maxPosition;
            this.lowThreshold = lowThreshold;
            this.highThreshold = highThreshold;
        }
    }

    public enum Gear {
        LOW,
        NEUTRAL,
        HIGH
    }

    private DoubleSolenoid m_solenoid;
    private WPI_CANCoder m_cancoder;
    private TJDriveModule m_drive;

    private double m_adjustment;
    private double m_lowThreshold;
    private double m_highThreshold;
    private double m_maxPosition;

    private Gear m_commandedGear;

    private static final double OUT_OF_BOUNDS_TIMEOUT = 1;
    private static final double FALCON_DISAGREEMENT_TIMEOUT = 1;
    private static final double FALCON_DISAGREEMENT_TOLERANCE = 0.1;
    private boolean m_disableEncoderPermanently;
    private long m_startOfOutOfBounds;
    private long m_startOfFalconDisagreement;

    public Shifter(ShifterParameters params, TJDriveModule drive) {
        m_drive = drive;
        m_solenoid = new DoubleSolenoid(params.pchId, params.pchType, params.outId, params.inId);
        //m_cancoder = new WPI_CANCoder(params.cancoderId);
        //m_cancoder.configFactoryDefault();

        m_adjustment = params.minPosition;
        m_lowThreshold = DriveUtils.mod(params.lowThreshold - m_adjustment, 360);
        m_highThreshold = DriveUtils.mod(params.highThreshold - m_adjustment, 360);
        m_maxPosition = DriveUtils.mod(params.maxPosition - m_adjustment, 360);

        m_commandedGear = Gear.NEUTRAL;

        m_disableEncoderPermanently = false;
        m_startOfOutOfBounds = Long.MAX_VALUE;
        m_startOfFalconDisagreement = Long.MAX_VALUE;
    }

    public void shiftToLow() {
        m_solenoid.set(Value.kReverse);
        m_commandedGear = Gear.LOW;
    }

    public void shiftToHigh() {
        m_solenoid.set(Value.kForward);
        m_commandedGear = Gear.HIGH;
    }

    public Gear getGear() {
        if (isEncoderConnected()) {
            return getGearFromEncoder();
        } else {
            return getCommandedGear();
        }
    }

    public Gear getCommandedGear() {
        return m_commandedGear;
    }

    public Gear getGearFromEncoder() {
        double position = getEncoderPosition();
        if (position <= m_lowThreshold) {
        return Gear.LOW;
        } else if (position >= m_highThreshold) {
        return Gear.HIGH;
        } else {
        return Gear.NEUTRAL;
        }
    }

    public Gear getGearFromFalconAgreement() {
        double cancoderSpeed = m_drive.getScaledSensorVelocity();
        double falconSpeedLow = m_drive.getFalconVelocityLowGear();
        double falconSpeedHigh = m_drive.getFalconVelocityHighGear();
        if (Math.abs(cancoderSpeed - falconSpeedLow) / cancoderSpeed <= FALCON_DISAGREEMENT_TOLERANCE) {
            return Gear.LOW;
        } else if (Math.abs(cancoderSpeed - falconSpeedHigh) / cancoderSpeed <= FALCON_DISAGREEMENT_TOLERANCE) {
            return Gear.HIGH;
        } else {
            return Gear.NEUTRAL;
        }
    }

    public boolean isEncoderConnected() {
        if (m_disableEncoderPermanently
                || m_cancoder.getLastError() == ErrorCode.SensorNotPresent
                || m_cancoder.getMagnetFieldStrength() == MagnetFieldStrength.BadRange_RedLED
                || m_cancoder.getMagnetFieldStrength() == MagnetFieldStrength.Invalid_Unknown
                || Robot.isSimulation()) {
            return false;
        }

        if (m_cancoder.getAbsolutePosition() > m_maxPosition) {
            if (m_startOfOutOfBounds == Long.MAX_VALUE) {
                m_startOfOutOfBounds = RobotController.getFPGATime();
            }
            else if (RobotController.getFPGATime() - m_startOfOutOfBounds >= OUT_OF_BOUNDS_TIMEOUT) {
                m_disableEncoderPermanently = true;
            }
            return false;
        } else {
            m_startOfOutOfBounds = Long.MAX_VALUE;
        }

        if (getGearFromFalconAgreement() != getGearFromEncoder()) {
            if (m_startOfFalconDisagreement == Long.MAX_VALUE) {
                m_startOfFalconDisagreement = RobotController.getFPGATime();
            } else if (RobotController.getFPGATime() - m_startOfFalconDisagreement >= FALCON_DISAGREEMENT_TIMEOUT) {
                m_disableEncoderPermanently = true;
                return false;
            }
        } else {
            m_startOfFalconDisagreement = Long.MAX_VALUE;
        }

        return m_cancoder.getLastError() == ErrorCode.SensorNotPresent
           || m_cancoder.getMagnetFieldStrength() == MagnetFieldStrength.BadRange_RedLED
           || m_cancoder.getMagnetFieldStrength() == MagnetFieldStrength.Invalid_Unknown
           || m_cancoder.getAbsolutePosition() > m_maxPosition;
    }
    
    private double getEncoderPosition() {
        return m_cancoder.getAbsolutePosition() - m_adjustment;
    }
}