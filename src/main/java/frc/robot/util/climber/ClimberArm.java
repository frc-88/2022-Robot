package frc.robot.util.climber;

import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.util.Vector2D;
import frc.robot.util.WrappedAngle;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class ClimberArm {
    
    private final WPI_TalonFX m_pivot;
    private final WPI_TalonFX m_telescope;

    private final String m_positionLabel;

    private ClimberCalibrationProcedure m_calibration; // May be null before instantiation.

    private SingleJointedArmSim m_pivotSim;
    private ElevatorSim m_telescopeSim;

    private static final double PIVOT_RATIO = 360. / (100. * 2048.); // Motor ticks to actual degrees
    private static final double TELESCOPE_RATIO = (2.6 * Math.PI) / (25. * 2048.); // Motor ticks to actual inches

    private static final double PIVOT_MIN_ANGLE = -50;
    private static final double PIVOT_MAX_ANGLE = 29;
    protected static final double TELESCOPE_MIN_HEIGHT = 27;
    private static final double TELESCOPE_MAX_HEIGHT = 60;

    private static final Vector2D PIVOT_LOCATION = Vector2D.createCartesianCoordinates(0, 13.123);
    private static final Vector2D HOOK_TOP_CENTER = Vector2D.createCartesianCoordinates(1.773, 0.754);
    private static final double HOOK_TOP_RADIUS = 2.154;

    private static final double HEIGHT_LIMIT = 66;
    private static final double FRONT_LIMIT = 32;
    private static final double BACK_LIMIT = -28;

    private static class MotorPreferences {
        private final DoublePreferenceConstant triggerCurrent;
        private final DoublePreferenceConstant triggerDuration;
        private final DoublePreferenceConstant continuousCurrent;
        private final DoublePreferenceConstant maxVelocity;
        private final DoublePreferenceConstant maxAcceleration;
        private final PIDPreferenceConstants pid;

        private List<WPI_TalonFX> motors;

        double ratio;

        public MotorPreferences(String prefix, double ratio) {
            this.ratio = ratio;

            triggerCurrent = new DoublePreferenceConstant(prefix + " Trigger Current", 80);
            triggerDuration = new DoublePreferenceConstant(prefix + " Trigger Duration", 0.002);
            continuousCurrent = new DoublePreferenceConstant(prefix + " Continuous Current", 10);
            maxVelocity = new DoublePreferenceConstant(prefix + " Max Velocity", 0);
            maxAcceleration = new DoublePreferenceConstant(prefix + " Max Acceleration", 0);
            pid = new PIDPreferenceConstants(prefix + " PID", 0, 0, 0, 0, 0, 0, 0);

            Consumer<Double> handler = (Double unused) -> updateController();
            triggerCurrent.addChangeHandler(handler);
            triggerDuration.addChangeHandler(handler);
            continuousCurrent.addChangeHandler(handler);
            maxVelocity.addChangeHandler(handler);
            maxAcceleration.addChangeHandler(handler);
            pid.addChangeHandler(handler);

            motors = new LinkedList<>();
        }

        public void registerMotor(WPI_TalonFX motor) {
            motors.add(motor);
            updateController();
        }

        private void updateController() {
            StatorCurrentLimitConfiguration config = new StatorCurrentLimitConfiguration(
                true,
                continuousCurrent.getValue(),
                triggerCurrent.getValue(),
                triggerDuration.getValue()
            );
            motors.forEach((WPI_TalonFX motor) -> {
                motor.configStatorCurrentLimit(config);
                motor.configMotionCruiseVelocity(convertActualVelocitytoMotorVelocity(maxVelocity.getValue(), ratio));
                motor.configMotionAcceleration(convertActualVelocitytoMotorVelocity(maxAcceleration.getValue(), ratio));
                motor.config_kP(0, pid.getKP().getValue());
                motor.config_kI(0, pid.getKI().getValue());
                motor.config_kD(0, pid.getKD().getValue());
                motor.config_kF(0, pid.getKF().getValue());
                motor.config_IntegralZone(0, pid.getIZone().getValue());
                motor.configMaxIntegralAccumulator(0, pid.getIMax().getValue());
            });
        }
    }

    private static MotorPreferences pivotPreferences;
    private static MotorPreferences telescopePreferences;

    private static boolean staticInitialized = false;

    private void staticInit() {
        pivotPreferences = new MotorPreferences("Climber Pivot", PIVOT_RATIO);
        telescopePreferences = new MotorPreferences("Climber Telescope", TELESCOPE_RATIO);

        staticInitialized = true;
    }

    public ClimberArm(String positionLabel, int pivotID, int telescopeID, boolean pivotInverted) {
        if (!staticInitialized) {
            staticInit();
        }

        m_pivot = new WPI_TalonFX(pivotID);
        m_telescope = new WPI_TalonFX(telescopeID);
        m_positionLabel = positionLabel;

        m_pivot.configFactoryDefault();
        m_telescope.configFactoryDefault();

        if (pivotInverted || Robot.isSimulation()) {
            m_pivot.setInverted(InvertType.InvertMotorOutput);
            m_telescope.setInverted(InvertType.None);
        } else {
            m_pivot.setInverted(InvertType.None);
            m_telescope.setInverted(InvertType.InvertMotorOutput);
        }

        m_pivot.setNeutralMode(NeutralMode.Brake);
        m_telescope.setNeutralMode(NeutralMode.Brake);

        m_pivot.configReverseSoftLimitThreshold(convertPivotActualPositionToMotor(PIVOT_MIN_ANGLE));
        m_pivot.configForwardSoftLimitThreshold(convertPivotActualPositionToMotor(PIVOT_MAX_ANGLE));
        m_telescope.configReverseSoftLimitThreshold(convertTelescopeActualPositionToMotor(TELESCOPE_MIN_HEIGHT));
        m_telescope.configForwardSoftLimitThreshold(convertTelescopeActualPositionToMotor(TELESCOPE_MAX_HEIGHT));

        pivotPreferences.registerMotor(m_pivot);
        telescopePreferences.registerMotor(m_telescope);

        m_pivotSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), 100., 0.15, Units.inchesToMeters(36), Units.degreesToRadians(PIVOT_MIN_ANGLE - 90), Units.degreesToRadians(PIVOT_MAX_ANGLE - 90), Units.lbsToKilograms(3), false);
        m_telescopeSim = new ElevatorSim(DCMotor.getFalcon500(1), 25., Units.lbsToKilograms(2), Units.inchesToMeters(2.6), Units.inchesToMeters(TELESCOPE_MIN_HEIGHT), Units.inchesToMeters(TELESCOPE_MAX_HEIGHT));
    }


    public void setPercentOutput(double pivotPercent, double telescopePercent) {
        m_pivot.set(TalonFXControlMode.PercentOutput, pivotPercent);
        m_telescope.set(TalonFXControlMode.PercentOutput, telescopePercent);
    }

    public void setMotionMagic(double pivotAngle, double telescopeHeight) {
        m_pivot.set(TalonFXControlMode.MotionMagic, convertPivotActualPositionToMotor(pivotAngle));
        m_telescope.set(TalonFXControlMode.MotionMagic, convertTelescopeActualPositionToMotor(telescopeHeight));
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

    public void coast() {
        m_pivot.setNeutralMode(NeutralMode.Coast);
        m_telescope.setNeutralMode(NeutralMode.Coast);
    }

    public void brake() {
        m_pivot.setNeutralMode(NeutralMode.Brake);
        m_telescope.setNeutralMode(NeutralMode.Brake);
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

    public Vector2D getPositionVector() {
        return getPositionVector(getPivotAngle(), getTelescopeHeight());
    }

    private static Vector2D getPositionVector(double pivotAngle, double telescopeHeight) {
        return Vector2D.createPolarCoordinates(telescopeHeight, new WrappedAngle(-pivotAngle + 90));
    }


    public static boolean isPositionInvalid(double pivotAngle, double telescopeHeight) {
        return doesPositionViolatePivotMin(pivotAngle, telescopeHeight)
                || doesPositionViolatePivotMax(pivotAngle, telescopeHeight)
                || doesPositionViolateTelescopeMin(pivotAngle, telescopeHeight)
                || doesPositionViolateTelescopeMax(pivotAngle, telescopeHeight)
                || doesPositionViolateHeightLimit(pivotAngle, telescopeHeight)
                || doesPositionViolateFrontExtension(pivotAngle, telescopeHeight)
                || doesPositionViolateBackExtension(pivotAngle, telescopeHeight);
    }

    public static Pair<Double, Double> getNearestValidPosition(double pivotAngle, double telescopeHeight) {
        if (doesPositionViolatePivotMin(pivotAngle, telescopeHeight)) {
            pivotAngle = PIVOT_MIN_ANGLE;
        }
        if (doesPositionViolatePivotMax(pivotAngle, telescopeHeight)) {
            pivotAngle = PIVOT_MAX_ANGLE;
        }
        if (doesPositionViolateTelescopeMin(pivotAngle, telescopeHeight)) {
            telescopeHeight = TELESCOPE_MIN_HEIGHT;
        }
        if (doesPositionViolateTelescopeMax(pivotAngle, telescopeHeight)) {
            telescopeHeight = TELESCOPE_MAX_HEIGHT;
        }
        // if (doesPositionViolateHeightLimit(pivotAngle, telescopeHeight)) {
        //     telescopeHeight = HEIGHT_LIMIT - HOOK_TOP_RADIUS - PIVOT_LOCATION.getX() .plus(HOOK_TOP_CENTER.rotate(-pivotAngle)).getY();
        // }
        // if (doesPositionViolateFrontExtension(pivotAngle, telescopeHeight)) {
        //     telescopeHeight = (FRONT_LIMIT - HOOK_TOP_RADIUS - PIVOT_LOCATION.plus(HOOK_TOP_CENTER.rotate(-pivotAngle)).getX()) / Math.cos(Math.toRadians(-pivotAngle + 90));
        // }
        // if (doesPositionViolateBackExtension(pivotAngle, telescopeHeight)) {
        //     telescopeHeight = (-BACK_LIMIT - HOOK_TOP_RADIUS + PIVOT_LOCATION.plus(HOOK_TOP_CENTER).rotate(-pivotAngle).getX()) / Math.abs(Math.cos(Math.toRadians(-pivotAngle + 90)));
        // }
        return new Pair<Double,Double>(pivotAngle, telescopeHeight);
    }

    private static boolean doesPositionViolatePivotMin(double pivotAngle, double telescopeHeight) {
        return pivotAngle < PIVOT_MIN_ANGLE;
    }

    private static boolean doesPositionViolatePivotMax(double pivotAngle, double telescopeHeight) {
        return pivotAngle > PIVOT_MAX_ANGLE;
    }

    private static boolean doesPositionViolateTelescopeMin(double pivotAngle, double telescopeHeight) {
        return telescopeHeight < TELESCOPE_MIN_HEIGHT;
    }

    private static boolean doesPositionViolateTelescopeMax(double pivotAngle, double telescopeHeight) {
        return telescopeHeight > TELESCOPE_MAX_HEIGHT;
    }

    private static boolean doesPositionViolateHeightLimit(double pivotAngle, double telescopeHeight) {
        return getHookTopCenterFromCenterGround(pivotAngle, telescopeHeight).getY() + HOOK_TOP_RADIUS > HEIGHT_LIMIT;
    }

    private static boolean doesPositionViolateFrontExtension(double pivotAngle, double telescopeHeight) {
        return getHookTopCenterFromCenterGround(pivotAngle, telescopeHeight).getX() + HOOK_TOP_RADIUS > FRONT_LIMIT;
    }

    private static boolean doesPositionViolateBackExtension(double pivotAngle, double telescopeHeight) {
        return getHookTopCenterFromCenterGround(pivotAngle, telescopeHeight).getX() - HOOK_TOP_RADIUS < BACK_LIMIT;
    }


    private static Vector2D getHookTopCenterFromCenterGround(double pivotAngle, double telescopeHeight) {
        return PIVOT_LOCATION.plus(getPositionVector(pivotAngle, telescopeHeight).plus(HOOK_TOP_CENTER.rotate(-pivotAngle)));
    }


    public void publishData() {
        SmartDashboard.putNumber(m_positionLabel + " Climber Pivot Angle", getPivotAngle());
        SmartDashboard.putNumber(m_positionLabel + " Climber Telescope Height", getTelescopeHeight());
        SmartDashboard.putNumber(m_positionLabel + " Climber X", getPositionVector().getX());
        SmartDashboard.putNumber(m_positionLabel + " Climber Y", getPositionVector().getY());
        SmartDashboard.putNumber(m_positionLabel + " Climber Telescope Current", m_telescope.getSupplyCurrent());
        SmartDashboard.putBoolean(m_positionLabel + " Climber Is Calibrated", isCalibrated());
    }


    public void simulate() {
        m_pivotSim.setInput(m_pivot.getSimCollection().getMotorOutputLeadVoltage());
        m_telescopeSim.setInput(m_telescope.getSimCollection().getMotorOutputLeadVoltage());

        m_pivotSim.update(0.02);
        m_telescopeSim.update(0.02);

        m_pivot.getSimCollection().setIntegratedSensorRawPosition((int)convertPivotActualPositionToMotor(Units.radiansToDegrees(m_pivotSim.getAngleRads()) - 90));
        m_pivot.getSimCollection().setIntegratedSensorVelocity((int)convertPivotActualVelocityToMotor(Units.radiansToDegrees(m_pivotSim.getVelocityRadPerSec()) - 90));
        m_telescope.getSimCollection().setIntegratedSensorRawPosition((int)convertTelescopeActualPositionToMotor(Units.metersToInches(m_telescopeSim.getPositionMeters())));
        m_telescope.getSimCollection().setIntegratedSensorVelocity((int)convertTelescopeActualVelocityToMotor(Units.metersToInches(m_telescopeSim.getVelocityMetersPerSecond())));
    }



    protected void enterCalibration() {
        m_pivot.configForwardSoftLimitEnable(false);
        m_pivot.configReverseSoftLimitEnable(false);
        m_telescope.configForwardSoftLimitEnable(false);
        m_telescope.configReverseSoftLimitEnable(false);
    }

    protected void finishCalibration() {
        m_pivot.setSelectedSensorPosition(convertPivotActualPositionToMotor(PIVOT_MAX_ANGLE));
        m_telescope.setSelectedSensorPosition(convertTelescopeActualPositionToMotor(TELESCOPE_MIN_HEIGHT));

        if (Robot.isSimulation()) {
            Matrix<N2, N1> pivotState = new Matrix<>(N2.instance, N1.instance);
            pivotState.set(0, 0, PIVOT_MAX_ANGLE - 90);
            pivotState.set(1, 0, 0);
            m_pivotSim.setState(pivotState);
        }

        m_pivot.configForwardSoftLimitEnable(true);
        m_pivot.configReverseSoftLimitEnable(true);
        m_telescope.configForwardSoftLimitEnable(true);
        m_telescope.configReverseSoftLimitEnable(true);
    }



    private static double convertPivotMotorPositionToActual(double motorPosition) {
        return convertMotorPositionToActualPosition(motorPosition, PIVOT_RATIO);
    }

    private static double convertPivotMotorVelocityToActual(double motorVelocity) {
        return convertMotorVelocityToActualVelocity(motorVelocity, PIVOT_RATIO);
    }

    private static double convertPivotActualPositionToMotor(double actualPosition) {
        return convertActualPositiontoMotorPosition(actualPosition, PIVOT_RATIO);
    }

    private static double convertPivotActualVelocityToMotor(double actualVelocity) {
        return convertActualVelocitytoMotorVelocity(actualVelocity, PIVOT_RATIO);
    }


    private static double convertTelescopeMotorPositionToActual(double motorPosition) {
        return convertMotorPositionToActualPosition(motorPosition, TELESCOPE_RATIO);
    }

    private static double convertTelescopeMotorVelocityToActual(double motorVelocity) {
        return convertMotorVelocityToActualVelocity(motorVelocity, TELESCOPE_RATIO);
    }

    private static double convertTelescopeActualPositionToMotor(double actualPosition) {
        return convertActualPositiontoMotorPosition(actualPosition, TELESCOPE_RATIO);
    }

    private static double convertTelescopeActualVelocityToMotor(double actualVelocity) {
        return convertActualVelocitytoMotorVelocity(actualVelocity, TELESCOPE_RATIO);
    }


    private static double convertMotorPositionToActualPosition(double motorPosition, double ratio) {
        return motorPosition * ratio;
    }

    private static double convertMotorVelocityToActualVelocity(double motorVelocity, double ratio) {
        return convertMotorPositionToActualPosition(motorVelocity, ratio) * 10.;
    }

    private static double convertActualPositiontoMotorPosition(double actualPosition, double ratio) {
        return actualPosition / ratio;
    }

    private static double convertActualVelocitytoMotorVelocity(double actualVelocity, double ratio) {
        return convertActualPositiontoMotorPosition(actualVelocity, ratio) * 0.1;
    }
}
