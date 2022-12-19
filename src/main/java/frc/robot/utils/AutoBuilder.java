package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot.AutoModes;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCubeCommand;

public class AutoBuilder {
    private RobotContainer robotContainer;
    private AutoModes autoMode;
    private SequentialCommandGroup autoCommand;
    private AutoPath startPath;

    public Command build() {
        autoCommand = new SequentialCommandGroup();

        switch (autoMode) {
            case AUTO_ONE:
                break;
        }

        autoCommand.beforeStarting(new InstantCommand(() -> robotContainer.drivetrainSubsystem.resetOdometer(startPath.getInitPose())));
        return autoCommand;
    }

    public void setRobotContainer(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public void setAutoMode(AutoModes autoMode) {
        this.autoMode = autoMode;
    }

    private void autoOne() {
        startPath = new AutoPath(robotContainer.drivetrainSubsystem, Constants.AutoConstants.pathOne);

        autoCommand.addCommands(
                startPath.getAutoPath().alongWith(
                        new IntakeCubeCommand(robotContainer.clawSubsystem).withTimeout(5.0)
                ),
                new InstantCommand(
                        () -> robotContainer.clawSubsystem.setFlywheelSetpoint(
                                Constants.ShooterConstants.SCALE_SHOT_SPEED), robotContainer.clawSubsystem),
                new WaitCommand(2.0),
                new InstantCommand(robotContainer.clawSubsystem::stopFlywheel)
        );
    }
}
