package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class CloseClaw extends CommandBase {
	private final ClawSubsystem clawSubsystem;

	public CloseClaw(ClawSubsystem clawSubsystem) {
		this.clawSubsystem = clawSubsystem;

		addRequirements(clawSubsystem);
	}

	@Override
	public void initialize() {
		clawSubsystem.closeClaw();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
