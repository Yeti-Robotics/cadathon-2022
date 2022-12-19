// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveForDistanceCommand extends CommandBase {
  private final DriveTrainSubsystem driveTrainSubsystem;
  private final double goal;
  private final double power;
  /** Creates a new DriveForDistanceCommand. */
  public DriveForDistanceCommand(DriveTrainSubsystem driveTrainSubsystem, double encoderGoal, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.goal = encoderGoal;
    this.power = Math.abs(power);
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrainSubsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (goal < 0){
      driveTrainSubsystem.drive(-power, 0.0);
    }else{
      driveTrainSubsystem.drive(power, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
