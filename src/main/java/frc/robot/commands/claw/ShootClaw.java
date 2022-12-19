package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ShootClaw extends CommandBase {
	private final ClawSubsystem clawSubsystem;

	public ShootClaw(ClawSubsystem clawSubsystem) {
		this.clawSubsystem = clawSubsystem;

		addRequirements(clawSubsystem);
	}

	@Override
	public void execute() {
		clawSubsystem.closeClaw();
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		clawSubsystem.stopMotors();
	}
}
