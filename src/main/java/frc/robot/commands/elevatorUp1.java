// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.elevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
public class elevatorUp1 extends CommandBase {
  private final ElevatorSubsystem elevatorSubsystem;
  public elevatorUp1(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem=elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.lift1(elevatorConstants.elevatorDistance);
    elevatorSubsystem.elevatorStop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
