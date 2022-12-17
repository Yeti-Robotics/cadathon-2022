package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot.AutoModes;
import frc.robot.RobotContainer;

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
}
