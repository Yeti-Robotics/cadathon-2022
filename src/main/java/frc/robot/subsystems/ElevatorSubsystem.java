package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
	private final CANSparkMax leftMotor = new CANSparkMax(ElevatorConstants.LEFT_MOTOR_PORT, MotorType.kBrushless);
	private final CANSparkMax rightMotor = new CANSparkMax(ElevatorConstants.RIGHT_MOTOR_PORT, MotorType.kBrushless);

	public ElevatorSubsystem() {
		leftMotor.follow(rightMotor);
		leftMotor.setInverted(true);
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
