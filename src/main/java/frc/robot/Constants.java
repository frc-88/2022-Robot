// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.util.drive.Shifter;

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

	/********************************************************************
	* 
    *                ____       _          
    *       --------/ __ \_____(_)   _____ 
    *      --------/ / / / ___/ / | / / _ \
    *     --------/ /_/ / /  / /| |/ /  __/
    *    --------/_____/_/  /_/ |___/\___/ 
	* 
	*/

	// Drive CAN IDs
	public static final int LEFT_MASTER_DRIVE_ID = 19;
	public static final int LEFT_FOLLOWER_DRIVE_ID = 18;
	public static final int RIGHT_MASTER_DRIVE_ID = 0;
	public static final int RIGHT_FOLLOWER_DRIVE_ID = 1;
	public static final int LEFT_DRIVE_ENCODER_ID = 0;
	public static final int RIGHT_DRIVE_ENCODER_ID = 19;

	// Drive Configuration
	public static final int NUM_DRIVE_MOTORS_PER_SIDE = 2;
	public static final double WHEEL_DIAMETER = 5.84375;
	public static final double LOW_GEAR_RATIO = (1. / 18.38 * (10. / 11.));
	public static final double HIGH_GEAR_RATIO = (1. / 8.50 * (10. / 11.));
	public static final double LOW_DRIVE_RATIO = LOW_GEAR_RATIO * (WHEEL_DIAMETER / 12.) * Math.PI;
	public static final double HIGH_DRIVE_RATIO = HIGH_GEAR_RATIO * (WHEEL_DIAMETER / 12.) * Math.PI;
	public static final double DRIVE_SENSOR_RATIO = (1. / ((WHEEL_DIAMETER / 12.) * Math.PI)) * 60./24.;
	public static final double DRIVE_LOW_STATIC_FRICTION_VOLTAGE = 0.2;
	public static final double DRIVE_HIGH_STATIC_FRICTION_VOLTAGE = 0.24;
	public static final double DRIVE_LEFT_LOW_EFFICIENCY = 1.025;
	public static final double DRIVE_LEFT_HIGH_EFFICIENCY = 1.03;
	public static final double DRIVE_RIGHT_LOW_EFFICIENCY = 1.03;
	public static final double DRIVE_RIGHT_HIGH_EFFICIENCY = 1.02;
	public static final double MAX_SPEED_LOW = 8.8;
	public static final double MAX_SPEED_HIGH = 16.5;
	public static final double WHEEL_BASE_WIDTH = 26.25 / 12.; // feet
	public static final double DRIVE_CURRENT_LIMIT = 300;
	
	public static final double METERS_TO_FEET = 3.28084;  // multiply your number by this value to convert to feet
	public static final double FEET_TO_METERS = 0.3048;  // multiply your number by this value to convert to meters

	public static final double MAX_TRAJ_VELOCITY = 6.0;
	public static final double MAX_TRAJ_ACCELERATION = 4.0;
	public static final double MAX_TRAJ_CENTRIP_ACC = 1.5;

	public static final Shifter.ShifterParameters LEFT_SHIFTER_CONSTANTS = new Shifter.ShifterParameters(PneumaticsModuleType.REVPH, 1, 5, 4, 18, 120, 240, 150, 210);
	public static final Shifter.ShifterParameters RIGHT_SHIFTER_CONSTANTS = new Shifter.ShifterParameters(PneumaticsModuleType.REVPH, 1, 3, 2, 1, 120, 240, 150, 210);

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
	public static final int SHOOTER_FLYWHEEL_ID = 8;
    public static final int HOOD_ID = 9;

	// Turret
	public static final int TURRET_MOTOR_ID = 13;
	public static final int TURRET_CANCODER_ID = 13;
	public static final double TURRET_GEAR_RATIO = 7.0 * 144.0 / 18;  // 56.0
	public static final double TURRET_CANCODER_GEAR_RATIO = 7.0 * 2.0 * 54.0 / 8.0;  // 94.5
    public static final double TURRET_COUNTS_PER_REV = TURRET_GEAR_RATIO * 2048.0;
	public static final double TURRET_SPIN_THRESHOLD = TURRET_COUNTS_PER_REV / 5.0;

	// Feeders
	public static final int FEEDER_CENTRALIZER_MOTOR_ID = 7;
    public static final int FEEDER_CENTRALIZER_BEAMBREAK = 3;

	public static final int FEEDER_CHAMBER_MOTOR_ID = 6;
	public static final int FEEDER_CHAMBER_BEAMBREAK = 2;

	// Intake
	public static final int INTAKE_ROLLER_ID = 1;
	public static final int INTAKE_ARM_ID = 12;
	public static final int INTAKE_IR_ID = 3;

	// Climber
	public static final int OUTER_CLIMBER_PIVOT_ID = 3;
	public static final int OUTER_CLIMBER_TELESCOPE_ID = 17;
	public static final int INNER_CLIMBER_PIVOT_ID = 16;
	public static final int INNER_CLIMBER_TELESCOPE_ID = 2;

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

	public static final int CAMERA_TILTER_SERVO_CHANNEL = 0;
	public static final double CAMERA_TILT_DOWN_COMMAND = 180;
	public static final double CAMERA_TILT_LEVEL_ANGLE = 55;
	public static final double CAMERA_TILT_UP_ANGLE = -70;
	public static final double CAMERA_TILT_DOWN_ANGLE = 30;

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
	// public static final String COPROCESSOR_ADDRESS = "127.0.0.1";
	public static final int COPROCESSOR_PORT = 5800;
	public static final double COPROCESSOR_TABLE_UPDATE_DELAY = 1.0 / 30.0;
	public static final double COPROCESSOR_PERIODIC_UPDATE_DELAY = 1.0 / 30.0;
	public static final double COPROCESSOR_PERIODIC_UPDATE_OFFSET = 1.0 / 60.0;
	public static final double COPROCESSOR_SLOW_PERIODIC_UPDATE_DELAY = 1.0 / 5.0;
	public static final double COPROCESSOR_SLOW_PERIODIC_UPDATE_OFFSET = 0.025;

}
