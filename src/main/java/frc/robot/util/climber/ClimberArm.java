package frc.robot.util.climber;

import java.util.LinkedList;
import java.util.List;
import java.util.Objects;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class ClimberArm {
    
    private final WPI_TalonFX m_pivot;
    private final WPI_TalonFX m_telescope;

    private final String m_positionLabel;

    private ClimberCalibrationProcedure m_calibration; // May be null before instantiation.

    private static final double PIVOT_RATIO = 360. / (100. * 2048.); // Motor ticks to actual degrees
    private static final double TELESCOPE_RATIO = (2.6 * Math.PI) / (25. * 2048.); // Motor ticks to actual inches

    private static final double PIVOT_MIN_ANGLE = -45;
    private static final double PIVOT_MAX_ANGLE = 30;
    protected static final double TELESCOPE_MIN_HEIGHT = 24;
    private static final double TELESCOPE_MAX_HEIGHT = 48;

    private static class PreferenceCurrentLimit {
        private final DoublePreferenceConstant triggerCurrent;
        private final DoublePreferenceConstant triggerDuration;
        private final DoublePreferenceConstant continuousCurrent;

        private List<WPI_TalonFX> motors;

        public PreferenceCurrentLimit(String prefix) {
            triggerCurrent = new DoublePreferenceConstant(prefix + " Trigger Current", 80);
            triggerDuration = new DoublePreferenceConstant(prefix + " Trigger Duration", 0.002);
            continuousCurrent = new DoublePreferenceConstant(prefix + " Continuous Current", 10);

            triggerCurrent.addChangeHandler((Double unused) -> updateCurrentLimit());
            triggerDuration.addChangeHandler((Double unused) -> updateCurrentLimit());
            continuousCurrent.addChangeHandler((Double unused) -> updateCurrentLimit());

            motors = new LinkedList<>();
        }

        public void registerMotor(WPI_TalonFX motor) {
            motors.add(motor);
            updateCurrentLimit();
        }

        private void updateCurrentLimit() {
            StatorCurrentLimitConfiguration config = new StatorCurrentLimitConfiguration(
                true,
                continuousCurrent.getValue(),
                triggerCurrent.getValue(),
                triggerDuration.getValue()
            );
            motors.forEach((WPI_TalonFX motor) -> motor.configStatorCurrentLimit(config));
        }
    }

    private static PreferenceCurrentLimit pivotCurrentLimit;
    private static PreferenceCurrentLimit telescopeCurrentLimit;

    private static boolean staticInitialized = false;

    private void staticInit() {
        pivotCurrentLimit = new PreferenceCurrentLimit("Pivot");
        telescopeCurrentLimit = new PreferenceCurrentLimit("Telescope");

        staticInitialized = true;
    }

    public ClimberArm(String positionLabel, int pivotID, int telescopeID) {
        if (!staticInitialized) {
            staticInit();
        }

        m_pivot = new WPI_TalonFX(pivotID);
        m_telescope = new WPI_TalonFX(telescopeID);
        m_positionLabel = positionLabel;

        m_pivot.configFactoryDefault();
        m_telescope.configFactoryDefault();

        m_pivot.setInverted(InvertType.InvertMotorOutput);
        m_telescope.setInverted(InvertType.None);

        m_pivot.setNeutralMode(NeutralMode.Brake);
        m_telescope.setNeutralMode(NeutralMode.Brake);

        m_pivot.configReverseSoftLimitThreshold(convertPivotActualPositionToMotor(PIVOT_MIN_ANGLE));
        m_pivot.configForwardSoftLimitThreshold(convertPivotActualPositionToMotor(PIVOT_MAX_ANGLE));
        m_telescope.configReverseSoftLimitThreshold(convertTelescopeActualPositionToMotor(TELESCOPE_MIN_HEIGHT));
        m_telescope.configForwardSoftLimitThreshold(convertTelescopeActualPositionToMotor(TELESCOPE_MAX_HEIGHT));

        pivotCurrentLimit.registerMotor(m_pivot);
        telescopeCurrentLimit.registerMotor(m_telescope);
    }


    public void setPercentOutput(double pivotPercent, double telescopePercent) {
        m_pivot.set(TalonFXControlMode.PercentOutput, pivotPercent);
        m_telescope.set(TalonFXControlMode.PercentOutput, telescopePercent);
    }


    public void resetCalibration() {
        // Doing this instead of instantiating in constructor to avoid passing
        // this object before it is constructed.
        if (Objects.isNull(m_calibration)) {
            m_calibration = new ClimberCalibrationProcedure(this);
        }

        m_calibration.resetCalibration();
    }

    public void calibrate() {
        // Doing this instead of instantiating in constructor to avoid passing
        // this object before it is constructed.
        if (Objects.isNull(m_calibration)) {
            m_calibration = new ClimberCalibrationProcedure(this);
        }

        m_calibration.runCalibration();
    }

    public boolean isCalibrated() {
        return Objects.nonNull(m_calibration) && m_calibration.isCalibrated();
    }


    public double getPivotAngle() {
        return convertPivotMotorPositionToActual(m_pivot.getSelectedSensorPosition());
    }

    public double getTelescopeHeight() {
        return convertTelescopeMotorPositionToActual(m_telescope.getSelectedSensorPosition());
    }

    public double getPivotVelocity() {
        return convertPivotMotorVelocityToActual(m_pivot.getSelectedSensorVelocity());
    }

    public double getTelescopeVelocity() {
        return convertTelescopeMotorVelocityToActual(m_telescope.getSelectedSensorVelocity());
    }


    public void publishData() {
        SmartDashboard.putNumber(m_positionLabel + " Climber Pivot Angle", getPivotAngle());
        SmartDashboard.putNumber(m_positionLabel + " Climber Telescope Height", getTelescopeHeight());
        SmartDashboard.putBoolean(m_positionLabel + " Climber Is Calibrated", isCalibrated());
    }



    protected void enterCalibration() {
        m_pivot.configForwardSoftLimitEnable(false);
        m_pivot.configReverseSoftLimitEnable(false);
        m_telescope.configForwardSoftLimitEnable(false);
        m_telescope.configReverseSoftLimitEnable(false);
    }

    protected void finishCalibration() {
        m_pivot.setSelectedSensorPosition(PIVOT_MAX_ANGLE);
        m_telescope.setSelectedSensorPosition(TELESCOPE_MIN_HEIGHT);

        m_pivot.configForwardSoftLimitEnable(true);
        m_pivot.configReverseSoftLimitEnable(true);
        m_telescope.configForwardSoftLimitEnable(true);
        m_telescope.configReverseSoftLimitEnable(true);
    }



    private double convertPivotMotorPositionToActual(double motorPosition) {
        return convertMotorPositionToActualPosition(motorPosition, PIVOT_RATIO);
    }

    private double convertPivotMotorVelocityToActual(double motorVelocity) {
        return convertMotorVelocityToActualVelocity(motorVelocity, PIVOT_RATIO);
    }

    private double convertPivotActualPositionToMotor(double actualPosition) {
        return convertActualPositiontoMotorPosition(actualPosition, PIVOT_RATIO);
    }

    private double convertPivotActualVelocityToMotor(double actualVelocity) {
        return convertActualVelocityToMotorVelocity(actualVelocity, PIVOT_RATIO);
    }


    private double convertTelescopeMotorPositionToActual(double motorPosition) {
        return convertMotorPositionToActualPosition(motorPosition, TELESCOPE_RATIO);
    }

    private double convertTelescopeMotorVelocityToActual(double motorVelocity) {
        return convertMotorVelocityToActualVelocity(motorVelocity, TELESCOPE_RATIO);
    }

    private double convertTelescopeActualPositionToMotor(double actualPosition) {
        return convertActualPositiontoMotorPosition(actualPosition, TELESCOPE_RATIO);
    }

    private double convertTelescopeActualVelocityToMotor(double actualVelocity) {
        return convertActualVelocityToMotorVelocity(actualVelocity, TELESCOPE_RATIO);
    }


    private double convertMotorPositionToActualPosition(double motorPosition, double ratio) {
        return motorPosition * ratio;
    }

    private double convertMotorVelocityToActualVelocity(double motorVelocity, double ratio) {
        return convertMotorPositionToActualPosition(motorVelocity, ratio) * 10.;
    }

    private double convertActualPositiontoMotorPosition(double actualPosition, double ratio) {
        return actualPosition / ratio;
    }

    private double convertActualVelocityToMotorVelocity(double actualVelocity, double ratio) {
        return convertActualPositiontoMotorPosition(actualVelocity, ratio) * 0.1;
    }
}
