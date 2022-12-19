package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
	private final WPI_TalonFX leftMotor = new WPI_TalonFX(ElevatorConstants.LEFT_MOTOR_PORT);
	private final WPI_TalonFX rightMotor = new WPI_TalonFX(ElevatorConstants.RIGHT_MOTOR_PORT);

	public ElevatorSubsystem() {
		leftMotor.follow(rightMotor);
		leftMotor.setInverted(InvertType.OpposeMaster);
	}

	public void moveUp() {
		rightMotor.set(ElevatorConstants.ELEVATOR_SPEED);
	}

	public void moveDown() {
		rightMotor.set(-ElevatorConstants.ELEVATOR_SPEED);
	}

	public void stop() {
		rightMotor.set(0);
	}
}
