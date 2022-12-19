package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
	// kForward for these pistons means they are open
	private final DoubleSolenoid leftPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
			ClawConstants.LEFT_PISTON_PORTS[0], ClawConstants.LEFT_PISTON_PORTS[1]);
	private final DoubleSolenoid rightPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
			ClawConstants.RIGHT_PISTON_PORTS[0], ClawConstants.RIGHT_PISTON_PORTS[1]);

	private final DoubleSolenoid upDownPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
			ClawConstants.UP_DOWN_PISTON_PORTS[0], ClawConstants.UP_DOWN_PISTON_PORTS[1]);

	private final WPI_TalonFX rightMotor = new WPI_TalonFX(ClawConstants.RIGHT_MOTOR_PORT);
	private final WPI_TalonFX leftMotor = new WPI_TalonFX(ClawConstants.LEFT_MOTOR_PORT);

	public ClawSubsystem() {
		// default to claw being open
		leftPiston.set(Value.kForward);
		rightPiston.set(Value.kForward);

		leftMotor.follow(rightMotor);
		leftMotor.setInverted(InvertType.OpposeMaster);
	}

	public void openClaw() {
		leftPiston.set(Value.kForward);
		rightPiston.set(Value.kForward);
	}

	public void closeClaw() {
		leftPiston.set(Value.kReverse);
		rightPiston.set(Value.kReverse);
	}

	public void shootClawMotors() {
		rightMotor.set(ClawConstants.MOTOR_SPEED);
	}

	public void intakeClawMotors() {
		rightMotor.set(-ClawConstants.MOTOR_SPEED);
	}

	public void stopMotors() {
		rightMotor.set(0);
	}
}
