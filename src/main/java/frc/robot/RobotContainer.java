// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.plaf.ButtonUI;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakePistonsCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...

    public DrivetrainSubsystem drivetrainSubsystem;
    public ElevatorSubsystem elevatorSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public IntakeCommand intakeCommand;
    public IntakePistonsCommand intakePistonsCommand;
    public OuttakeCommand outtakeCommand;
    public Joystick joystick;
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {

        drivetrainSubsystem = new DrivetrainSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        intakeCommand = new IntakeCommand(intakeSubsystem);
        intakePistonsCommand = new IntakePistonsCommand(intakeSubsystem);
        outtakeCommand = new OuttakeCommand(intakeSubsystem);
        joystick = new Joystick(0);

        drivetrainSubsystem.setDefaultCommand(
            new RunCommand(
                () -> drivetrainSubsystem.cheesyDrive(getLeftY(), getRightX()), drivetrainSubsystem));
        
        // Configure the button bindings
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
        // Add button to command mappings here.
        // See https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html

        setJoystickButtonWhenPressed(joystick, 1, new IntakeCommand(intakeSubsystem));
        setJoystickButtonWhileHeld(joystick, 2, new OuttakeCommand(intakeSubsystem));

    }


    private double getLeftY() {
        return -joystick.getRawAxis(0);
    }
    private double getLeftX() {
        return joystick.getRawAxis(1);
      }
    
      private double getRightY() {
        return -joystick.getRawAxis(2);
      }
    
      private double getRightX() {
        return joystick.getRawAxis(3);
      }
    
      private void setJoystickButtonWhenPressed(Joystick joystick, int button, CommandBase command) {
        new JoystickButton(joystick, button).whenPressed(command);
    }

    private void setJoystickButtonWhileHeld(Joystick joystick, int button, CommandBase command) {
        new JoystickButton(joystick, button).whileHeld(command);
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An ExampleCommand will run in autonomous
        return new InstantCommand();
    }
}
