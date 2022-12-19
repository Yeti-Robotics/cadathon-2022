package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorDown extends CommandBase {
	private final ElevatorSubsystem elevatorSubsystem;

	public MoveElevatorDown(ElevatorSubsystem elevatorSubsystem) {
		this.elevatorSubsystem = elevatorSubsystem;
		addRequirements(elevatorSubsystem);
	}

	@Override
	public void execute() {
		this.elevatorSubsystem.moveDown();
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.elevatorSubsystem.stop();
	}
}
