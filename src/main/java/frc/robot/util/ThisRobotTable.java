package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Turret;
import frc.robot.util.climber.ClimberArm;
import frc.robot.util.coprocessortable.ChassisInterface;
import frc.robot.util.coprocessortable.CoprocessorTable;
import frc.robot.util.coprocessortable.MessageTimer;

public class ThisRobotTable extends CoprocessorTable {
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

    private ClimberArm outerArm;
    private ClimberArm innerArm;

    private Intake intake;
    private Turret turret;
    private Sensors sensors;
    private Hood hood;
    private SwerveDrive drive;

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

    public ThisRobotTable(
        ChassisInterface chassis, String address, int port, double updateInterval,
            ClimberArm outerArm, ClimberArm innerArm,
            Intake intake,
            Turret turret,
            Sensors sensors,
            Hood hood) {
        super(chassis, address, port, updateInterval);

        this.outerArm = outerArm;
        this.innerArm = innerArm;
        this.intake = intake;
        this.turret = turret;
        this.sensors = sensors;
        this.hood = hood;
        this.drive = (SwerveDrive)chassis;

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

    private double gsToMetersPerSecondSquared(double gs) {
        return gs * 9.81;
    }
    
    @Override
    public void update() {
        drive.updateOdometry();
        super.update();
        
        sendImu(
            Units.degreesToRadians(sensors.ahrs_navx.getYaw()),
            Units.degreesToRadians(sensors.ahrs_navx.getRate()),
            gsToMetersPerSecondSquared(sensors.ahrs_navx.getRawAccelX()),
            gsToMetersPerSecondSquared(sensors.ahrs_navx.getRawAccelY())
        );
    }

    public void updateSlow() {
        if (!isConnected()) {
            return;
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
        setHoodState(this.hood.isUp());

        SmartDashboard.putNumber("ROS Ping", NetworkTableInstance.getDefault().getEntry("/ROS/status/tunnel/ping").getDouble(0.0));
    }

    private void setHoodState(boolean state) {
        hoodStateEntry.setBoolean(state);
        hoodStateUpdate.setDouble(getTime());
    }

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
}
