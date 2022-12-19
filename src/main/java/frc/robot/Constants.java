// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final class DrivetrainConstants {
		public static final int RIGHT_MOTOR_1_PORT = 0;

		public static final int RIGHT_MOTOR_2_PORT = 1;

		public static final int RIGHT_MOTOR_3_PORT = 2;

		public static final int LEFT_MOTOR_1_PORT = 3;

		public static final int LEFT_MOTOR_2_PORT = 4;

		public static final int LEFT_MOTOR_3_PORT = 5;
	}

	public static final class ElevatorConstants {
		public static final int LEFT_MOTOR_PORT = 6;

		public static final int RIGHT_MOTOR_PORT = 7;

		/** Guesswork */
		public static final double ELEVATOR_SPEED = 0.3;
	}

	public static final class ClawConstants {
		public static final int[] LEFT_PISTON_PORTS = { 0, 1 };

		public static final int[] RIGHT_PISTON_PORTS = { 2, 3 };

		public static final int[] UP_DOWN_PISTON_PORTS = { 4, 5 };

		public static final int RIGHT_MOTOR_PORT = 8;

		public static final int LEFT_MOTOR_PORT = 9;

		/** Guesswork */
		public static final double MOTOR_SPEED = 0.3;
	}

	public static final class IOConstants {
		public static final int JOYSTICK_PORT = 0;
	}
}
