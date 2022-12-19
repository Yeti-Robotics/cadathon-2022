package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystem;


public class IntakeCubeCommand extends CommandBase {
    private final ClawSubsystem clawSubsystem;

    public IntakeCubeCommand(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;

        addRequirements(this.clawSubsystem);
    }

    @Override
    public void initialize() {
        clawSubsystem.setFlywheelSetpoint(Constants.ShooterConstants.INTAKE_SPEED);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return clawSubsystem.isCubeInClaw();
    }

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.stopFlywheel();
    }
}
