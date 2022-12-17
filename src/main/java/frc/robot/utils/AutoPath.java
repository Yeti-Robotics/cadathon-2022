package frc.robot.utils;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoPath {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private PathPlannerTrajectory trajectory;
    private PPSwerveControllerCommand swerveControllerCommand;
    private double startVel = 0.0;
    private double endVel = 0.0;
    private double maxVel = AutoConstants.MAX_VELOCITY;
    private double maxAccel = AutoConstants.MAX_ACCEL;

    public AutoPath(DrivetrainSubsystem drivetrainSubsystem, String trajectoryFile) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.trajectory = PathPlanner.loadPath(trajectoryFile, maxVel, maxAccel);

        generateAutoPathCommand();
    }

    public AutoPath(DrivetrainSubsystem drivetrainSubsystem, String trajectoryFile, double maxVel, double maxAccel) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.maxVel = maxVel;
        this.maxAccel= maxAccel;
        this.trajectory = PathPlanner.loadPath(trajectoryFile, maxVel, maxAccel);

        generateAutoPathCommand();
    }

    private void generateAutoPathCommand() {
        swerveControllerCommand = new PPSwerveControllerCommand(
            trajectory,
            drivetrainSubsystem::getPose,
            DriveConstants.DRIVE_KINEMATICS,
            drivetrainSubsystem.getxController(),
            drivetrainSubsystem.getyController(),
            drivetrainSubsystem.getThetaController(),
            drivetrainSubsystem::setDesiredStates,
            drivetrainSubsystem);
    }

    /**
     * DO NOT use if it is a part of a command group
     * Use getAutoPath() instead
     */
    public void runAutoPath() {
        new ScheduleCommand(swerveControllerCommand.beforeStarting(
            new InstantCommand(() -> drivetrainSubsystem.resetOdometer(getInitPose())))
        );
    }

    public Command getAutoPath() {
        return swerveControllerCommand;
    }

    public Pose2d getInitPose() {
        return trajectory.getInitialPose();
    }

    public Pose2d getEndPose() {
        return trajectory.getEndState().poseMeters;
    }

    public PathPlannerTrajectory getTrajectory() {
        return trajectory;
    }

    public double getPathDuration() {
        return trajectory.getTotalTimeSeconds();
    }
}
