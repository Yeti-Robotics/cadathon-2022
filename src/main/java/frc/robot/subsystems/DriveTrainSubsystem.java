// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainSubsystem extends SubsystemBase {
  private final WPI_TalonFX left1TalonFX;
  private final WPI_TalonFX left2TalonFX;
  private final WPI_TalonFX right1TalonFX;
  private final WPI_TalonFX right2TalonFX;

  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem() {
    left1TalonFX = new WPI_TalonFX(1);
    left2TalonFX = new WPI_TalonFX(2);
    right1TalonFX = new WPI_TalonFX(3);
    right2TalonFX = new WPI_TalonFX(4);
  }
 
  public void drive(double leftpower,double rightpower){
    left1TalonFX.set(leftpower);
    left2TalonFX.set(leftpower);
    right1TalonFX.set(rightpower);
    right2TalonFX.set(rightpower);
  }
  public void stop(){
    left1TalonFX.set(0.0);
    left2TalonFX.set(0.0);
    right1TalonFX.set(0.0);
    right2TalonFX.set(0.0);
  }
public void resetEncoders(){
  left1TalonFX.setSelectedSensorPosition(0.0);
  right1TalonFX.setSelectedSensorPosition(0.0);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
