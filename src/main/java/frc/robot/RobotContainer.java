// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.IntakeCubeCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utils.controller.ButtonHelper;
import frc.robot.utils.controller.ControllerContainer;
import frc.robot.utils.controller.MultiButton;
import frc.robot.utils.controller.MultiButton.RunCondition;
import frc.robot.Constants.ShooterConstants;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final ControllerContainer controllerContainer;
    private final ButtonHelper buttonHelper;

    public final DrivetrainSubsystem drivetrainSubsystem;
    public final ClawSubsystem clawSubsystem;
    public final ClimberSubsystem climberSubsystem;

    public RobotContainer()
    {
        controllerContainer = new ControllerContainer();
        buttonHelper = new ButtonHelper(controllerContainer.getControllers());

        drivetrainSubsystem = new DrivetrainSubsystem(controllerContainer.get(0));
        drivetrainSubsystem.setDriveMode(DrivetrainSubsystem.DriveMode.CHEEZY);
        clawSubsystem = new ClawSubsystem();
        climberSubsystem = new ClimberSubsystem();

        configureButtonBindings();
    }
    
    
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        buttonHelper.setController(0);
        buttonHelper.createButton(11, 0, new InstantCommand(drivetrainSubsystem::toggleShifter), RunCondition.WHEN_PRESSED);

        buttonHelper.createButton(12, 0, new InstantCommand(clawSubsystem::toggleClawPosition), RunCondition.WHEN_PRESSED);

        buttonHelper.createButton(1, 0, new IntakeCubeCommand(clawSubsystem), RunCondition.WHILE_HELD);
        buttonHelper.createButton(6, 0,
                new StartEndCommand(
                () -> clawSubsystem.setFlywheelSetpoint(-Constants.ShooterConstants.INTAKE_SPEED), clawSubsystem::stopFlywheel, clawSubsystem),
                RunCondition.WHILE_HELD);

        buttonHelper.createButton(2, 0,
                new InstantCommand(() -> clawSubsystem.setFlywheelSetpoint(ShooterConstants.SCALE_SHOT_SPEED), clawSubsystem),
                RunCondition.WHEN_PRESSED);
        buttonHelper.createButton(3, 0,
                new InstantCommand(() -> clawSubsystem.setFlywheelSetpoint(ShooterConstants.SWITCH_SHOT_SPEED), clawSubsystem),
                RunCondition.WHEN_PRESSED);
        buttonHelper.createButton(7, 0, new InstantCommand(clawSubsystem::stopFlywheel), RunCondition.WHEN_PRESSED);
        buttonHelper.createButton(8, 0, new InstantCommand(clawSubsystem::stopFlywheel), RunCondition.WHEN_PRESSED);

        buttonHelper.createButton(1, 1,
                new StartEndCommand(climberSubsystem::climbDown, climberSubsystem::stopClimb, climberSubsystem),
                RunCondition.WHILE_HELD);
        buttonHelper.createButton(6, 1,
                new StartEndCommand(climberSubsystem::climbUp, climberSubsystem::stopClimb, climberSubsystem),
                RunCondition.WHILE_HELD);

        buttonHelper.createButton(10, 0,
                new InstantCommand(() -> {
                    if (MultiButton.syncLayer == 1) {
                        buttonHelper.setAllLayers(0);
                        return;
                    }
                    buttonHelper.setAllLayers(1);
                }),
                RunCondition.WHEN_PRESSED);
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An ExampleCommand will run in autonomous
        Command autoCommand = null;
        return autoCommand;
    }
}
