// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static).
 * Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    ///////////////////////////////////////////////////////
    // DRIVETRAIN
    ///////////////////////////////////////////////////////

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.56515; // 22.25 inches
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.57785; // 22.75 inches

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 0; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 0; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(198.6);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 18; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 19; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 18; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(142.1);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 8; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 9; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 8; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(56.6);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 10; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 11; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 10;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(77.0);
	
	public static final double MAX_TRAJ_VELOCITY = 6.0;
	public static final double MAX_TRAJ_ACCELERATION = 4.0;
	public static final double MAX_TRAJ_CENTRIP_ACC = 1.5;

	public static final int DRIVE_SPEED_EXP_XBOX = 2;
	public static final int DRIVE_TURN_EXP_XBOX = 2;
	public static final int DRIVE_SPEED_EXP_FRSKY = 2;
	public static final int DRIVE_TURN_EXP_FRSKY = 2;
	public static final double XBOX_DEADBAND = 0.12;
	public static final double FRSKY_DEADBAND = 0.07;
	public static final double CHEESY_DRIVE_MIN_TURN = 0.4;
	public static final double CHEESY_DRIVE_MAX_TURN = 0.9;
	public static final double CHEESY_DRIVE_FORCE_LOW_MIN_TURN = 0.6;
	public static final double CHEESY_DRIVE_FORCE_LOW_MAX_TURN = 1.5;
	
	// Shooter & Hood
	public static final int SHOOTER_FLYWHEEL_ID = 15;
	public static final int SHOOTER_FLYWHEEL_FOLLOWER_ID = 14;
    public static final int HOOD_ID = 13;

	// Turret
	public static final int TURRET_MOTOR_ID = 5;
	public static final int TURRET_CANCODER_ID = 5;
	public static final double TURRET_GEAR_RATIO = 7.0 * 148.0 / 20;  // 51.8
	public static final double TURRET_CANCODER_GEAR_RATIO = 7.0 * 2.0 * 7.0;  // 98.0
	public static final double TURRET_COUNTS_PER_REV = TURRET_GEAR_RATIO * 2048.0;
	public static final double TURRET_SPIN_THRESHOLD = TURRET_COUNTS_PER_REV / 5.0;

	// Feeders
	public static final int CENTRALIZER_ID = 7;
	public static final int CHAMBER_ID = 6;

	// Intake
	public static final int INTAKE_ROLLER_ID = 0;
	public static final int INTAKE_ARM_ID = 12;

	// Climber
	public static final int OUTER_CLIMBER_PIVOT_ID = 17;
	public static final int OUTER_CLIMBER_TELESCOPE_ID = 4;
	public static final int INNER_CLIMBER_PIVOT_ID = 3;
	public static final int INNER_CLIMBER_TELESCOPE_ID = 16;

	// Sensors
	public static final int SENSORS_COAST_BUTTON_ID = 0;

	public static final int STORAGE_PRESSURE_SENSOR_CHANNEL = 1;
	public static final int WORKING_PRESSURE_SENSOR_CHANNEL = 0;
	public final static double PRESSURE_DIFFERENCE_TARGET = 5;
	public final static double WORKING_PRESSURE_WARNING = 50;
	public final static double LEAK_WARNING = 0.01;
	public final static double PRESSURE_SENSOR_MIN_VOLTAGE = 0.4;
	public final static double PRESSURE_SENSOR_MAX_VOLTAGE = 4;

	public static final int COLOR_SENSOR_PROXIMITY_THRESHOLD = 1500;
    public static final double COLOR_SENSOR_BLUE_CARGO_BLUE_THRESHOLD = 0;
    public static final double COLOR_SENSOR_BLUE_CARGO_RED_THRESHOLD = 0;
    public static final double COLOR_SENSOR_BLUE_CARGO_GREEN_THRESHOLD = 0;

	public static final int CAMERA_TILTER_SERVO_CHANNEL = 2;
	public static final double CAMERA_TILT_DOWN_COMMAND = 180;
	public static final double CAMERA_TILT_LEVEL_ANGLE = 55;
	
	public static final double CAMERA_TILT_UP_ANGLE = -70;
	public static final double CAMERA_TILT_DOWN_ANGLE = 10;

	// Controllers
	public static final int DRIVER_CONTROLLER_ID = 0;
	public static final int BUTTON_BOX_ID = 1;
	public static final int TEST_CONTROLLER_ID = 2;
	public static final int TEST_CONTROLLER_2_ID = 3;
    
	// Field constants
	public static final double SHOOTING_ZONE_INNER_RADIUS_METERS = 0.2;
	public static final double SHOOTING_ZONE_OUTER_RADIUS_METERS = 0.3;
	public static final double FIELD_VISION_TARGET_HEIGHT = 102;
	public static final double FIELD_UPPER_HUB_RADIUS = 20.0;

	// ROS Interface
	public static final String COPROCESSOR_ADDRESS = "10.0.88.44";
	public static final String COPROCESSOR_ADDRESS_SIMULATED = "127.0.0.1";
	public static final int COPROCESSOR_PORT = 5800;
	public static final double COPROCESSOR_TABLE_UPDATE_DELAY = 1.0 / 30.0;
	public static final double COPROCESSOR_PERIODIC_UPDATE_DELAY = 1.0 / 30.0;
	public static final double COPROCESSOR_PERIODIC_UPDATE_OFFSET = 1.0 / 60.0;
	public static final double COPROCESSOR_SLOW_PERIODIC_UPDATE_DELAY = 1.0 / 5.0;
	public static final double COPROCESSOR_SLOW_PERIODIC_UPDATE_OFFSET = 0.025;

}
