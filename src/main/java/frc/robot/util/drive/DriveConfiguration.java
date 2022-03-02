package frc.robot.util.drive;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.Constants;

public class DriveConfiguration {
    public TJDriveModuleConfiguration left, right;

    private TalonFXConfiguration _masterConfiguration;
    private TalonFXConfiguration _followerConfiguration;

    public DriveConfiguration() {
        left = new TJDriveModuleConfiguration();
        right = new TJDriveModuleConfiguration();

        left.master = Constants.LEFT_MASTER_DRIVE_ID;
        left.followers = new int[] { Constants.LEFT_FOLLOWER_DRIVE_ID };

        right.master = Constants.RIGHT_MASTER_DRIVE_ID;
        right.followers = new int[] { Constants.RIGHT_FOLLOWER_DRIVE_ID };

        /* Master */

        _masterConfiguration = new TalonFXConfiguration();

        _masterConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        _masterConfiguration.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        _masterConfiguration.neutralDeadband = 0.00;
        _masterConfiguration.voltageCompSaturation = 12;

        left.masterConfiguration = _masterConfiguration;
        right.masterConfiguration = _masterConfiguration;

        // /* Followers */
        _followerConfiguration = new TalonFXConfiguration();

        _followerConfiguration.neutralDeadband = 0.00;

        left.followerConfiguration = _followerConfiguration;
        right.followerConfiguration = _followerConfiguration;

        // /* General Settings */
        left.neutralMode = NeutralMode.Brake;
        left.invertMotor = true;
        left.invertSensor = true;
        left.enableVoltageCompensation = true;
        right.neutralMode = NeutralMode.Brake;
        right.invertMotor = true;
        right.invertSensor = true;
        right.enableVoltageCompensation = true;
    }
}