package frc.robot.util.coprocessor.networktables;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.Vector2D;
import frc.robot.util.WrappedAngle;
import frc.robot.util.climber.ClimberArm;
import frc.robot.util.coprocessor.ChassisInterface;
import frc.robot.util.coprocessor.MessageTimer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Turret;


public class SwerveTable extends CoprocessorTable {
    private SwerveDrive swerve;
    private AHRS imu;

    final int left_outer_climber_joint = 0;
    final int left_inner_climber_joint = 1;
    final int right_outer_climber_joint = 2;
    final int right_inner_climber_joint = 3;
    final int left_outer_climber_hook_joint = 4;
    final int left_inner_climber_hook_joint = 5;
    final int right_outer_climber_hook_joint = 6;
    final int right_inner_climber_hook_joint = 7;
    final int intake_joint = 8;
    final int turret_joint = 9;
    final int camera_joint = 10;

    private NetworkTable targetTable;
    private NetworkTableEntry targetEntryDist;
    private NetworkTableEntry targetEntryAngle;
    private NetworkTableEntry targetEntryProbability;
    private NetworkTableEntry targetEntryUpdate;
    private double targetDistance = 0.0;
    private double targetAngle = 0.0;
    private double targetProbability = 0.0;
    private MessageTimer targetTimer = new MessageTimer(1_000_000);


    private ClimberArm outerArm;
    private ClimberArm innerArm;

    private Intake intake;
    private Turret turret;
    private Sensors sensors;

    private SwerveDrive drive;

    private final double kGravity = 9.81;

    private NetworkTableEntry fieldRelativeEntry;

    private NetworkTable moduleRootTable;
    private NetworkTableEntry moduleNumEntry;
    private ArrayList<NetworkTable> moduleTables = new ArrayList<>();

    private NetworkTable imuTable;
    private NetworkTableEntry imuEntryTx;  // filtered roll angle
    private NetworkTableEntry imuEntryTy;  // filtered pitch angle
    private NetworkTableEntry imuEntryTz;  // filtered yaw angle
    private NetworkTableEntry imuEntryVz;  // yaw rate
    private NetworkTableEntry imuEntryAx;  // linear accel x
    private NetworkTableEntry imuEntryAy;  // linear accel y
    private NetworkTableEntry imuEntryUpdate;

    private NetworkTable hoodTable;
    private NetworkTableEntry hoodStateEntry;
    private NetworkTableEntry hoodStateUpdate;

    private NetworkTable shooterTargetTable;
    private NetworkTableEntry shooterTargetEntryDist;
    private NetworkTableEntry shooterTargetEntryAngle;
    private NetworkTableEntry shooterTargetEntryProbability;
    private NetworkTableEntry shooterTargetEntryUpdate;
    private double shooterDistance = 0.0;
    private double shooterAngle = 0.0;
    private double shooterProbability = 1.0;
    private MessageTimer shooterTimer = new MessageTimer(1_000_000);

    private NetworkTable barTable;
    private NetworkTableEntry barEntryDist;
    private NetworkTableEntry barEntryAngle;
    private NetworkTableEntry barEntryCount;
    private NetworkTableEntry barEntryUpdate;
    private double barDistance = 0.0;
    private double barAngle = 0.0;
    private int barCount = 0;
    private MessageTimer barTimer = new MessageTimer(1_000_000);

    private NetworkTable resetPoseToLimelightTable;
    private NetworkTableEntry resetPoseToLimelightUpdate;

    private NetworkTable shooterTable;
    private NetworkTableEntry shooterEntryDist;
    private NetworkTableEntry shooterEntryAngle;
    private NetworkTableEntry shooterEntryCounter;
    private NetworkTableEntry shooterEntrySpeed;
    private int shotCounter = 0;

    private NetworkTable targetConfigTable;
    private NetworkTableEntry targetConfigEntryShotCorrection;
    private NetworkTableEntry targetConfigEntryStationaryShotProbability;
    private NetworkTableEntry targetConfigEntryMovingShotProbability;
    private NetworkTableEntry targetConfigEntryLimelightFineTuning;
    private NetworkTableEntry targetConfigEntryCargoMarauding;
    private NetworkTableEntry targetConfigEntryResetToLimelight;
    private NetworkTableEntry targetConfigEntryUpdate;

    public SwerveTable(SwerveDrive swerve, String address, int port, double updateInterval, 
            ClimberArm outerArm, ClimberArm innerArm,
            Intake intake,
            Turret turret,
            Sensors sensors) {
        super((ChassisInterface)swerve, address, port, updateInterval);

        this.outerArm = outerArm;
        this.innerArm = innerArm;
        this.intake = intake;
        this.turret = turret;
        this.sensors = sensors;

        this.swerve = swerve;
        this.imu = swerve.getNavX();

        targetTable = rootTable.getSubTable("target");
        targetEntryDist = targetTable.getEntry("distance");
        targetEntryAngle = targetTable.getEntry("heading");
        targetEntryProbability = targetTable.getEntry("probability");
        targetEntryUpdate = targetTable.getEntry("update");
        targetEntryUpdate.addListener(this::targetCallback, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        
        fieldRelativeEntry = rootTable.getEntry("field_relative");
        fieldRelativeEntry.addListener(this::fieldRelativeCallback, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        moduleRootTable = rootTable.getSubTable("modules");
        moduleNumEntry = moduleRootTable.getEntry("num");
        for (int index = 0; index < this.swerve.getNumModules(); index++) {
            moduleTables.add(moduleRootTable.getSubTable(String.valueOf(index)));
        }
        moduleNumEntry.setDouble((double)this.swerve.getNumModules());

        imuTable = rootTable.getSubTable("imu");
        imuEntryTx = imuTable.getEntry("tx");
        imuEntryTy = imuTable.getEntry("ty");
        imuEntryTz = imuTable.getEntry("tz");
        imuEntryVz = imuTable.getEntry("vz");
        imuEntryAx = imuTable.getEntry("ax");
        imuEntryAy = imuTable.getEntry("ay");
        imuEntryUpdate = imuTable.getEntry("update");

        hoodTable = getRootTable().getSubTable("hood");
        hoodStateEntry = hoodTable.getEntry("state");
        hoodStateUpdate = hoodTable.getEntry("update");

        shooterTargetTable = rootTable.getSubTable("target");
        shooterTargetEntryDist = shooterTargetTable.getEntry("distance");
        shooterTargetEntryAngle = shooterTargetTable.getEntry("heading");
        shooterTargetEntryProbability = shooterTargetTable.getEntry("probability");
        shooterTargetEntryUpdate = shooterTargetTable.getEntry("update");
        shooterTargetEntryUpdate.addListener(this::shooterTargetCallback, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        barTable = rootTable.getSubTable("bar");
        barEntryDist = barTable.getEntry("distance");
        barEntryAngle = barTable.getEntry("angle");
        barEntryCount = barTable.getEntry("count");
        barEntryUpdate = barTable.getEntry("update");
        barEntryUpdate.addListener(this::barCallback, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        resetPoseToLimelightTable = rootTable.getSubTable("resetToLimelight");
        resetPoseToLimelightUpdate = resetPoseToLimelightTable.getEntry("update");

        shooterTable = rootTable.getSubTable("shooter");
        shooterEntryCounter = shooterTable.getEntry("counter");
        shooterEntryAngle = shooterTable.getEntry("angle");
        shooterEntryDist = shooterTable.getEntry("distance");
        shooterEntrySpeed = shooterTable.getEntry("speed");

        targetConfigTable = rootTable.getSubTable("target_config");
        targetConfigEntryShotCorrection = targetConfigTable.getEntry("enable_shot_correction");
        targetConfigEntryStationaryShotProbability = targetConfigTable.getEntry("enable_stationary_shot_probability");
        targetConfigEntryMovingShotProbability = targetConfigTable.getEntry("enable_moving_shot_probability");
        targetConfigEntryLimelightFineTuning = targetConfigTable.getEntry("enable_limelight_fine_tuning");
        targetConfigEntryCargoMarauding = targetConfigTable.getEntry("enable_marauding");
        targetConfigEntryResetToLimelight = targetConfigTable.getEntry("enable_reset_to_limelight");
        targetConfigEntryUpdate = targetConfigTable.getEntry("update");
    }

    @Override
    public void update() {
        super.update();

        updateImu();
        // updateModules();
    }

    public void updateSlow() {
        SwerveModule[] modules = this.swerve.getModules();
        for (int index = 0; index < modules.length; index++) {
            SwerveModule module = modules[index];
            setJointPosition(index, module.getSteerAngle());
        }

        Vector2D outerLeftArmVector = outerArm.getPositionVector();
        Vector2D outerRightArmVector = outerLeftArmVector;
        Vector2D innerLeftArmVector = innerArm.getPositionVector();
        Vector2D innerRightArmVector = innerLeftArmVector;

        // outerLeftArmVector
        setJointPosition(
            left_outer_climber_joint,
            convertClimberPivotAngle(outerLeftArmVector.getAngle())
        );

        setJointPosition(
            left_outer_climber_hook_joint,
            convertClimberTelescopeHeight(outerLeftArmVector.getMagnitude())
        );

        // innerLeftArmVector
        setJointPosition(
            left_inner_climber_joint,
            convertClimberPivotAngle(innerLeftArmVector.getAngle())
        );

        setJointPosition(
            left_inner_climber_hook_joint,
            convertClimberTelescopeHeight(innerLeftArmVector.getMagnitude())
        );

        // outerRightArmVector
        setJointPosition(
            right_outer_climber_joint,
            convertClimberPivotAngle(outerRightArmVector.getAngle())
        );

        setJointPosition(
            right_outer_climber_hook_joint,
            convertClimberTelescopeHeight(outerRightArmVector.getMagnitude())
        );

        // innerRightArmVector
        setJointPosition(
            right_inner_climber_joint,
            convertClimberPivotAngle(innerRightArmVector.getAngle())
        );

        setJointPosition(
            right_inner_climber_hook_joint,
            convertClimberTelescopeHeight(innerRightArmVector.getMagnitude())
        );

        // intake
        setJointPosition(
            intake_joint,
            convertIntakeAngle(intake.isDeployLimitTriggered(), intake.isStowLimitTriggered())
        );

        // turret
        setJointPosition(
            turret_joint,
            convertTurretAngle(turret.getFacing())
        );

        // camera
        setJointPosition(
            camera_joint,
            convertCameraTiltAngle(sensors.getCameraTilterAngle())
        );

        // hood
        SmartDashboard.putNumber("ROS Ping", NetworkTableInstance.getDefault().getEntry("/ROS/status/tunnel/ping").getDouble(0.0));
        // setHoodState(this.hood.isUp());
    }

    // private void setHoodState(boolean state) {
    //     hoodStateEntry.setBoolean(state);
    //     hoodStateUpdate.setDouble(getTime());
    // }

    public void resetPoseToLimelight() {
        resetPoseToLimelightUpdate.setDouble(getTime());
    }

    public void signalShot(double shotAngle, double shotDistance, double shotSpeed) {
        shooterEntryCounter.setDouble(shotCounter);
        shotCounter++;
        shooterEntryAngle.setDouble(shotAngle);
        shooterEntryDist.setDouble(shotDistance);
        shooterEntrySpeed.setDouble(shotSpeed);
    }

    public double getCameraTiltCommand() {
        return Math.toDegrees(getJointCommand(camera_joint));
    }

    private double convertClimberPivotAngle(WrappedAngle pivotAngle) {
        return Math.toRadians(-pivotAngle.asDouble() + 90.0);
    }

    private double convertClimberTelescopeHeight(double telescopeHeight) {
        return Units.inchesToMeters(telescopeHeight - ClimberArm.TELESCOPE_MIN_HEIGHT);
    }

    private double convertIntakeAngle(boolean isDeployLimit, boolean isStowLimit) {
        double angle = 0.0;
        if (!isDeployLimit && !isStowLimit) {
            angle = -45.0;
        }
        else if (isDeployLimit) {
            angle = 0.0;
        }
        else if (isStowLimit) {
            angle = -90.0;
        }
        return Math.toRadians(angle);
    }

    private double convertTurretAngle(double turretAngle) {
        return Math.toRadians(turretAngle);
    }

    private double convertCameraTiltAngle(Rotation2d cameraAngle) {
        return cameraAngle.getRadians();
    }

    private void shooterTargetCallback(EntryNotification notification) {
        shooterDistance = shooterTargetEntryDist.getDouble(0.0);
        shooterAngle = shooterTargetEntryAngle.getDouble(0.0);
        shooterProbability = shooterTargetEntryProbability.getDouble(1.);
        SmartDashboard.getEntry("Shot Probability").setDouble(shooterProbability);
        shooterTimer.reset();
    }

    private void barCallback(EntryNotification notification) {
        barDistance = barEntryDist.getDouble(0.0);
        barAngle = barEntryAngle.getDouble(0.0);
        barCount = (int)barEntryCount.getDouble(0.0);
        SmartDashboard.getEntry("Bar angle (deg)").setDouble(Units.radiansToDegrees(barAngle));
        SmartDashboard.getEntry("Bar distance (m)").setDouble(barDistance);
        barTimer.reset();
    }

    public double getShooterDistance() {
        return shooterDistance;
    }
    public double getShooterAngle() {
        return shooterAngle;
    }
    public double getShooterProbability() {
        return shooterProbability;
    }
    public boolean isShooterTargetValid() {
        return shooterTimer.isActive();
    }


    public double getBarDistance() {
        return barDistance;
    }
    public double getBarAngle() {
        return barAngle;
    }
    public int getBarCount() {
        return barCount;
    }
    public boolean isBarValid() {
        return barTimer.isActive();
    }

    public void setTargetConfig(int enable_shot_correction, int enable_stationary_shot_probability, int enable_moving_shot_probability, int enable_limelight_fine_tuning, int enable_marauding, int enable_reset_to_limelight)
    {
        targetConfigEntryShotCorrection.setDouble((double)enable_shot_correction);
        targetConfigEntryStationaryShotProbability.setDouble((double)enable_stationary_shot_probability);
        targetConfigEntryMovingShotProbability.setDouble((double)enable_moving_shot_probability);
        targetConfigEntryLimelightFineTuning.setDouble((double)enable_limelight_fine_tuning);
        targetConfigEntryCargoMarauding.setDouble((double)enable_marauding);
        targetConfigEntryResetToLimelight.setDouble((double)enable_reset_to_limelight);
        targetConfigEntryUpdate.setDouble(getTime());
    }

    public void setEnableCargoMarauding(boolean enable_marauding)
    {
        setTargetConfig(
            -1, 
            -1, 
            -1, 
            -1, 
            enable_marauding ? 1 : 0, 
            -1
        );
    }

    public void enableCargoMarauding() {
        setEnableCargoMarauding(true);
    }

    public void disableCargoMarauding() {
        setEnableCargoMarauding(false);
    }


    // private void updateModules() {
    //     for (int index = 0; index < this.swerve.getNumModules(); index++) {
    //         DiffSwerveModule module = this.swerve.getModule(index);
    //         NetworkTable moduleTable = moduleTables.get(index);
    //         moduleTable.getEntry("wheel_velocity").setDouble(module.getWheelVelocity());
    //         moduleTable.getEntry("azimuth_velocity").setDouble(module.getAzimuthVelocity());
    //         moduleTable.getEntry("azimuth").setDouble(module.getModuleAngle());
    //         moduleTable.getEntry("hi_voltage").setDouble(module.getHiMeasuredVoltage());
    //         moduleTable.getEntry("hi_voltage_ref").setDouble(module.getHiNextVoltage());
    //         moduleTable.getEntry("hi_velocity").setDouble(module.getHiRadiansPerSecond());
    //         moduleTable.getEntry("lo_voltage").setDouble(module.getLoMeasuredVoltage());
    //         moduleTable.getEntry("lo_voltage_ref").setDouble(module.getLoNextVoltage());
    //         moduleTable.getEntry("lo_velocity").setDouble(module.getLoRadiansPerSecond());
    //     }
    // }

    public void updateImu()
    {
        imuEntryTx.setDouble(Units.degreesToRadians(imu.getRoll()));
        imuEntryTy.setDouble(Units.degreesToRadians(imu.getPitch()));
        imuEntryTz.setDouble(Units.degreesToRadians(imu.getYaw()));
        imuEntryVz.setDouble(Units.degreesToRadians(imu.getRate()));
        imuEntryAx.setDouble(imu.getWorldLinearAccelX() * kGravity);
        imuEntryAy.setDouble(imu.getWorldLinearAccelY() * kGravity);
        imuEntryUpdate.setDouble(getTime());
    }

    private void targetCallback(EntryNotification notification) {
        targetDistance = targetEntryDist.getDouble(0.0);
        targetAngle = targetEntryAngle.getDouble(0.0);
        targetProbability = targetEntryProbability.getDouble(0.0);
        targetTimer.reset();
    }

    private void fieldRelativeCallback(EntryNotification notification) {
        // boolean value = notification.getEntry().getBoolean(false);
    }

    public double getTargetDistance() {
        return targetDistance;
    }
    public double getTargetAngle() {
        return targetAngle;
    }
    public double getTargetProbability() {
        return targetProbability;
    }
    public boolean isTargetValid() {
        return targetTimer.isActive();
    }
}
