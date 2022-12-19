// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final CANSparkMax m_leftMotor;
  private final CANSparkMax m_rightMotor;

  public ElevatorSubsystem() {
    m_leftMotor = new CANSparkMax(1, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(2, MotorType.kBrushless);
  }
  public void lift1(double speed){
    m_leftMotor.set(10);
    m_rightMotor.follow(m_leftMotor);
  }
  public void lift2(double speed){
    m_leftMotor.set(10);
    m_rightMotor.set(9);
  }
  public void elevatorStop(){
    m_leftMotor.set(0);
    m_rightMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
