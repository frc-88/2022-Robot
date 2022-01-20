// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
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
    public static final PneumaticsModuleType SHIFTER_LEFT_PCM_TYPE = PneumaticsModuleType.REVPH;
	public static final int SHIFTER_LEFT_PCM = 1;
	public static final int SHIFTER_LEFT_OUT = 0;
	public static final int SHIFTER_LEFT_IN = 15;
    public static final PneumaticsModuleType SHIFTER_RIGHT_PCM_TYPE = PneumaticsModuleType.REVPH;
	public static final int SHIFTER_RIGHT_PCM = 1;
	public static final int SHIFTER_RIGHT_OUT = 1;
	public static final int SHIFTER_RIGHT_IN = 14;
	public static final int LEFT_SHIFTER_ENCODER_ID = 20;
	public static final int RIGHT_SHIFTER_ENCODER_ID = 21;

	// Drive Configuration
	public static final int NUM_DRIVE_MOTORS_PER_SIDE = 2;
	public static final double WHEEL_DIAMETER = 5.84375;
	public static final double LOW_GEAR_RATIO = (1. / 18.38);
	public static final double HIGH_GEAR_RATIO = (1. / 8.50);
	public static final double LOW_DRIVE_RATIO = LOW_GEAR_RATIO * (WHEEL_DIAMETER/12.) * Math.PI;
	public static final double HIGH_DRIVE_RATIO = HIGH_GEAR_RATIO * (WHEEL_DIAMETER/12.) * Math.PI;
	public static final double DRIVE_SENSOR_RATIO = (1. / ((WHEEL_DIAMETER/12.) * Math.PI)) * 7.5;
	public static final double DRIVE_LOW_STATIC_FRICTION_VOLTAGE = 0.2;
	public static final double DRIVE_HIGH_STATIC_FRICTION_VOLTAGE = 0.24;
	public static final double DRIVE_LEFT_LOW_EFFICIENCY = 1.025;
	public static final double DRIVE_LEFT_HIGH_EFFICIENCY = 1.03;
	public static final double DRIVE_RIGHT_LOW_EFFICIENCY = 1.03;
	public static final double DRIVE_RIGHT_HIGH_EFFICIENCY = 1.02;
	public static final double MAX_SPEED_LOW = 8.8;
	public static final double MAX_SPEED_HIGH = 16.5;
	public static final double WHEEL_BASE_WIDTH = (25. + 5./16.) / 12.; // feet
	public static final double DRIVE_CURRENT_LIMIT = 300;

	public static final double LEFT_SHIFTER_ENCODER_MIN = 120;
	public static final double LEFT_LOW_GEAR_ENCODER_THRESHOLD = 150;
	public static final double LEFT_HIGH_GEAR_ENCODER_THRESHOLD = 210;
	public static final double LEFT_SHIFTER_ENCODER_MAX = 240;
	public static final double RIGHT_SHIFTER_ENCODER_MIN = 120;
	public static final double RIGHT_LOW_GEAR_ENCODER_THRESHOLD = 150;
	public static final double RIGHT_HIGH_GEAR_ENCODER_THRESHOLD = 210;
	public static final double RIGHT_SHIFTER_ENCODER_MAX = 240;

	public static final int DRIVE_SPEED_EXP = 2;
	public static final int DRIVE_TURN_EXP = 2;
	public static final double DRIVE_JOYSTICK_DEADBAND = 0.12;
	public static final double CHEESY_DRIVE_MIN_TURN = 0.4;
	public static final double CHEESY_DRIVE_MAX_TURN = 0.9;
	public static final double CHEESY_DRIVE_FORCE_LOW_MIN_TURN = 0.6;
	public static final double CHEESY_DRIVE_FORCE_LOW_MAX_TURN = 1.5;

	// Sensors
	public static final I2C.Port I2C_ONBOARD = I2C.Port.kOnboard;

	public static final int STORAGE_PRESSURE_SENSOR_CHANNEL = 0;
	public static final int WORKING_PRESSURE_SENSOR_CHANNEL = 1;
	public final static double PRESSURE_DIFFERENCE_TARGET = 5;
	public final static double WORKING_PRESSURE_WARNING = 50;
	public final static double LEAK_WARNING = 0.01;
	public final static double PRESSURE_SENSOR_MIN_VOLTAGE = 0.4;
	public final static double PRESSURE_SENSOR_MAX_VOLTAGE = 4;

}
