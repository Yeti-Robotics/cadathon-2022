// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
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

        public static final double HIGH_GEAR_RATIO = 5.47; // jvn
        public static final double LOW_GEAR_RATIO = 16.09; // jvn

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

        public static final double X_CONTROLLER_P = 2.9;
        public static final double Y_CONTROLLER_P = 2.9;
        public static final double THETA_CONTROLLER_P = 3.0;
    }

    public static final class ShooterConstants {
        public static final int LEFT_MOTOR = 5;
        public static final int RIGHT_MOTOR = 6;
        public static final SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT_CONFIGURATION =
                new SupplyCurrentLimitConfiguration(true, 40, 50, 0.3);
        public static final StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT_CONFIGURATION =
                new StatorCurrentLimitConfiguration(true, 40, 50, 0.3);

        public static final double FLYWHEEL_KP = 1.0;
        public static final double FLYWHEEL_KI = 1.0;
        public static final double FLYWHEEL_KD = 1.0;



        public static final int[] SOLENOID_CLAW = {2, 4};

        public static final int CUBE_SENSOR = 0;
    }

    public static final class TalonFXConstants {
        public static final int ENCODER_RESOLUTION = 2048;

        public static final double VOLTAGE_COMP = 11.0;
    }

    public static final class PneumaticConstants {
        public static final PneumaticsModuleType deviceType = PneumaticsModuleType.CTREPCM;
    }
}

