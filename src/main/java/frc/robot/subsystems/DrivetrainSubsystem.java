package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
	private final WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(DrivetrainConstants.RIGHT_MOTOR_1_PORT);
	private final WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(DrivetrainConstants.RIGHT_MOTOR_2_PORT);
	private final WPI_TalonSRX rightMotor3 = new WPI_TalonSRX(DrivetrainConstants.RIGHT_MOTOR_3_PORT);
	private final WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(DrivetrainConstants.LEFT_MOTOR_1_PORT);
	private final WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(DrivetrainConstants.LEFT_MOTOR_2_PORT);
	private final WPI_TalonSRX leftMotor3 = new WPI_TalonSRX(DrivetrainConstants.LEFT_MOTOR_3_PORT);

	private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2, rightMotor3);
	private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2, leftMotor3);

	private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

	public DrivetrainSubsystem() {
		rightMotors.setInverted(true);
		leftMotors.setInverted(false);

		drive.setDeadband(0.05);
	}

	public void cheesyDrive(double straight, double turn) {
		drive.curvatureDrive(straight, -turn, true);
	}

	public void stop() {
		drive.stopMotor();
	}
}
