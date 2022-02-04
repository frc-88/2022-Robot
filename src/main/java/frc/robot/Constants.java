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
	public static final int LEFT_MASTER_DRIVE_ID = 0;
	public static final int LEFT_FOLLOWER_DRIVE_ID = 1;
	public static final int RIGHT_MASTER_DRIVE_ID = 15;
	public static final int RIGHT_FOLLOWER_DRIVE_ID = 14;
	public static final int LEFT_DRIVE_ENCODER_ID = 0;
	public static final int RIGHT_DRIVE_ENCODER_ID = 15;

	// Drive Configuration
	public static final int NUM_DRIVE_MOTORS_PER_SIDE = 2;
	public static final double WHEEL_DIAMETER = 5.84375;
	public static final double LOW_GEAR_RATIO = (1. / 18.38);
	public static final double HIGH_GEAR_RATIO = (1. / 8.50);
	public static final double LOW_DRIVE_RATIO = LOW_GEAR_RATIO * (WHEEL_DIAMETER / 12.) * Math.PI;
	public static final double HIGH_DRIVE_RATIO = HIGH_GEAR_RATIO * (WHEEL_DIAMETER / 12.) * Math.PI;
	public static final double DRIVE_SENSOR_RATIO = (1. / ((WHEEL_DIAMETER / 12.) * Math.PI)) * 7.5;
	public static final double DRIVE_LOW_STATIC_FRICTION_VOLTAGE = 0.2;
	public static final double DRIVE_HIGH_STATIC_FRICTION_VOLTAGE = 0.24;
	public static final double DRIVE_LEFT_LOW_EFFICIENCY = 1.025;
	public static final double DRIVE_LEFT_HIGH_EFFICIENCY = 1.03;
	public static final double DRIVE_RIGHT_LOW_EFFICIENCY = 1.03;
	public static final double DRIVE_RIGHT_HIGH_EFFICIENCY = 1.02;
	public static final double MAX_SPEED_LOW = 8.8;
	public static final double MAX_SPEED_HIGH = 16.5;
	public static final double DRIVE_CURRENT_LIMIT = 300;
	public static final double WHEEL_BASE_WIDTH = (25. + 5. / 16.) / 12.; // feet
	public static final double MAX_TRAJ_VELOCITY = 16.0;
	public static final double MAX_TRAJ_ACCELERATION = 8.0;
	public static final double MAX_TRAJ_CENTRIP_ACC = 2.5;

	public static final Shifter.ShifterParameters LEFT_SHIFTER_CONSTANTS = new Shifter.ShifterParameters(PneumaticsModuleType.REVPH, 1, 0, 15, 20, 120, 240, 150, 210);
	public static final Shifter.ShifterParameters RIGHT_SHIFTER_CONSTANTS = new Shifter.ShifterParameters(PneumaticsModuleType.REVPH, 1, 1, 14, 21, 120, 240, 150, 210);

	public static final int DRIVE_SPEED_EXP = 2;
	public static final int DRIVE_TURN_EXP = 2;
	public static final double DRIVE_JOYSTICK_DEADBAND = 0.12;
	public static final double CHEESY_DRIVE_MIN_TURN = 0.4;
	public static final double CHEESY_DRIVE_MAX_TURN = 0.9;
	public static final double CHEESY_DRIVE_FORCE_LOW_MIN_TURN = 0.6;
	public static final double CHEESY_DRIVE_FORCE_LOW_MAX_TURN = 1.5;

	// Sensors
	public static final int STORAGE_PRESSURE_SENSOR_CHANNEL = 0;
	public static final int WORKING_PRESSURE_SENSOR_CHANNEL = 1;
	public final static double PRESSURE_DIFFERENCE_TARGET = 5;
	public final static double WORKING_PRESSURE_WARNING = 50;
	public final static double LEAK_WARNING = 0.01;
	public final static double PRESSURE_SENSOR_MIN_VOLTAGE = 0.4;
	public final static double PRESSURE_SENSOR_MAX_VOLTAGE = 4;
    public static final double BLUE_CARGO_BLUE_THRESHOLD = 0;
    public static final double BLUE_CARGO_RED_THRESHOLD = 0;
    public static final double BLUE_CARGO_GREEN_THRESHOLD = 0;
    
	// Shooter
	public static final int SHOOTER_FLYWHEEL_ID = 0;
    public static final int SHOOTER_HOOD_ID = 0;
    public static final double SHOOTER_DEFAULT_P = 0;
    public static final double SHOOTER_DEFAULT_I = 0;
    public static final double SHOOTER_DEFAULT_D = 0;
    public static final double SHOOTER_DEFAULT_F = 0;
}
