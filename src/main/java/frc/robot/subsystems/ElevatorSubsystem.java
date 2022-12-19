// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private WPI_TalonFX elevatorFalcon;

  private final TrapezoidProfile.Constraints constrants;
  private final ProfiledPIDController elevatorPID;

  public enum ElevatorHeight {
    SCALE, SWITCH
  }

  public ElevatorSubsystem() {


    elevatorFalcon = new WPI_TalonFX(6);
    elevatorFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    constrants = new TrapezoidProfile.Constraints(1.75, 0.75); //these are sample values
    elevatorPID = new ProfiledPIDController(1.3, 0.0, 0.7, constrants); //These are filler values - would need Sysid to get accurate values
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    if(DriverStation.getMatchTime() >= 30 && getElevatorDistance() > 4){ //placeholder fory
      elevatorStop();
    }
  }

  public void elevatorUp() {
    elevatorFalcon.set(ControlMode.PercentOutput, 0.5);
  }

  public void elevatorUp(double speed) {
    elevatorFalcon.set(ControlMode.PercentOutput, speed);
  }

  public void elevatorDown() {
    elevatorFalcon.set(ControlMode.PercentOutput, 0.5);
  }

  public void elevatorDown(double speed) {
    elevatorFalcon.set(ControlMode.PercentOutput, speed);
  }

  public void elevatorStop() {
    elevatorFalcon.set(ControlMode.PercentOutput, 0);
  }

  public void resetElevatorEncoder() {
    elevatorFalcon.setSelectedSensorPosition(0);
  }

  public double getElevatorDistance() {
    return elevatorFalcon.getSelectedSensorPosition() * (1 / 2048);
  }

}
