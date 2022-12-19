// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import java.util.Map;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double MOTOR_VOLTAGE_COMP = 11.0;
    public static final class OIConstants {
        public static final Map<Integer, ControllerType> CONTROLLERS = Map.of(
                0, ControllerType.CUSTOM
        );
        public static final int CONTROLLER_COUNT = CONTROLLERS.size();

        public enum ControllerType {
            CUSTOM,
            XBOX
        }
    }

    public static final class DrivetrainConstants {
        public static final double DEADBAND = 0.0525;

        public static final int LEFT_FALCON_1 = 1; // front relative to robot
        public static final int LEFT_FALCON_2 = 2; // rear relative to robot

        public static final int RIGHT_FALCON_1 = 3; // front relative to robot
        public static final int RIGHT_FALCON_2 = 4; // rear relative to robot

        public static final int[] SOLENOID_SHIFTER = {0, 1};

        public static final double HIGH_GEAR_RATIO = 5.47; // motor to output (5.47 : 1)
        public static final double LOW_GEAR_RATIO = 16.09; // motor to output (16.09 : 1)

        public static final double WHEEL_DIAMETER_IN = 4.0;
        public static final double WHEEL_DIAMETER_M = Units.inchesToMeters(WHEEL_DIAMETER_IN);  // 0.1016
        public static final double WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * Math.PI; // 0.3192

        public static final double DISTANCE_PER_PULSE_HIGH_GEAR =
                WHEEL_CIRCUMFERENCE_M / (TalonFXConstants.ENCODER_RESOLUTION * HIGH_GEAR_RATIO); // 0.0000246613
        public static final double DISTANCE_PER_PULSE_LOW_GEAR =
                WHEEL_CIRCUMFERENCE_M / (TalonFXConstants.ENCODER_RESOLUTION * LOW_GEAR_RATIO);
    }

    public static final class AutoConstants {
        /**
         * Max velocity in meters per second
         */
        public static final double MAX_VELOCITY = 5.0;
        /**
         * Max acceleration in meters per second squared
         */
        public static final double MAX_ACCEL = 8.0;

        public static final double AUTO_KS = 0.800699;
        public static final double AUTO_KV = 1.9414;
        public static final double AUTO_KA = 0.55335;
        public static final double AUTO_P = 2.9854;

        public static final double TRACK_WIDTH = 0.80692;
        public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);

        public static final double RAMSETE_B = 2.0;
        public static final double RAMSETE_ZETA = 0.7;
        public static final RamseteController RAMSETE_CONTROLLER = new RamseteController(RAMSETE_B, RAMSETE_ZETA);

        public static final String pathOne = "pathOne";
    }

    public static final class ShooterConstants {
        public static final int LEFT_MOTOR = 5;
        public static final int RIGHT_MOTOR = 6;
        public static final int PIVOT_MOTOR = 8;
        public static final SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT_CONFIGURATION =
                new SupplyCurrentLimitConfiguration(true, 40, 50, 0.3);
        public static final StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT_CONFIGURATION =
                new StatorCurrentLimitConfiguration(true, 40, 50, 0.3);

        public static final double FLYWHEEL_KP = 0.25;
        public static final double FLYWHEEL_KI = 0.0;
        public static final double FLYWHEEL_KD = 0.0;
        /*
         * kV and kA values calculated from https://www.reca.lc/flywheel?currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=85&flywheelMomentOfInertia=%7B%22s%22%3A0%2C%22u%22%3A%22in2%2Albs%22%7D&flywheelRadius=%7B%22s%22%3A0%2C%22u%22%3A%22in%22%7D&flywheelRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Reduction%22%7D&flywheelWeight=%7B%22s%22%3A1.5%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22Falcon%20500%22%7D&motorRatio=%7B%22magnitude%22%3A1.3333333333333333%2C%22ratioType%22%3A%22Step-up%22%7D&projectileRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&projectileWeight=%7B%22s%22%3A1.5%2C%22u%22%3A%22kg%22%7D&shooterMomentOfInertia=%7B%22s%22%3A8%2C%22u%22%3A%22in2%2Albs%22%7D&shooterRadius=%7B%22s%22%3A4%2C%22u%22%3A%22in%22%7D&shooterTargetSpeed=%7B%22s%22%3A4000%2C%22u%22%3A%22rpm%22%7D&shooterWeight=%7B%22s%22%3A1%2C%22u%22%3A%22lbs%22%7D&useCustomFlywheelMoi=1&useCustomShooterMoi=0
         */
        public static final double FLYWHEEL_KS = 0.43; // Volts
        public static final double FLYWHEEL_KV = 0.13; // Volts * s / Meter
        public static final double FLYWHEEL_KA = 0.0; // Volts * s^2 / Meter
        public static final double FLYWHEEL_ACCEL = 4.0; // m/s

        public static final double FLYWHEEL_GEAR_RATIO = 2; // motor to output (2 : 1...)
        public static final double FLYWHEEL_DIAMETER_IN = 4.0;
        public static final double FLYWHEEL_DIAMETER_M = Units.inchesToMeters(FLYWHEEL_DIAMETER_IN); // 0.1016
        public static final double FLYWHEEL_CIRCUMFERENCE_M = FLYWHEEL_DIAMETER_M * Math.PI;
        public static final double METER_PER_PULSE =
                FLYWHEEL_CIRCUMFERENCE_M * FLYWHEEL_GEAR_RATIO / TalonFXConstants.ENCODER_RESOLUTION;
        public static final double PULSE_PER_METER = 1.0 / METER_PER_PULSE;

        public static final int FORWARD_LIMIT = 100000;
        public static final int REVERSE_LIMIT = 0;

        public static final int STARTING_POSITION = 5;
        public static final int SHOOTING_POSITION = 60000;
        public static final int INTAKE_POSITION = 99995;

        public static final int PIVOT_MAX_VEL = 7500;
        public static final int PIVOT_MAX_ACCEL = 12000;
        public static final int PIVOT_ACCEL_SMOOTHING = 3;

        public static final double PIVOT_KF = 0.5;
        public static final double PIVOT_KP = 2.0;
        public static final double PIVOT_KI = 0.20;
        public static final double PIVOT_KD = 0.25;

        public static final int[] SOLENOID_CLAW = {2, 3};
        public static final int[] SOLENOID_PIVOT = {4, 5};

        public static final int CUBE_SENSOR = 0;

        /*
            All speeds are in meters / second
         */
        public static final double MAX_SPEED = 28.0;
        public static final double INTAKE_SPEED = 8.0;
        public static final double SWITCH_SHOT_SPEED = 15.0;
        public static final double SCALE_SHOT_SPEED = 23.0;
    }

    public static final class ClimberConstants {
        public static final int CLIMBER_MOTOR = 7;

        public static final double CLIMBER_SPEED = 0.7;

        public static final int FORWARD_LIMIT = 10000;
        public static final int REVERSE_LIMIT = 0;
    }

    public static final class TalonFXConstants {
        public static final int ENCODER_RESOLUTION = 2048;

        public static final double VOLTAGE_COMP = 11.0;
    }

    public static final class PneumaticConstants {
        public static final PneumaticsModuleType deviceType = PneumaticsModuleType.CTREPCM;
    }

    public static final class GameConstants {
        /**
         * Dimensions of the game pieces in inches
         * { Width x Length x Height }
         */
        public static final double[] CUBE_DIMENSIONS = { 13, 13, 11 };
        /**
         * Weight of game piece in kilograms
         */
        public static final double CUBE_WEIGHT = 1.5;
    }
}

