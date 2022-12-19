// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  CANSparkMax leftIntakeSparkMax;
  CANSparkMax rightIntakeSparkMax;
  DoubleSolenoid leftIntakePiston;
  DoubleSolenoid rightIntakePiston;


  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    leftIntakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
    rightIntakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
    leftIntakeSparkMax = new CANSparkMax(1, MotorType.kBrushless);
    rightIntakeSparkMax = new CANSparkMax(2, MotorType.kBrushless);
    rightIntakeSparkMax.setInverted(true);
  }
  public void extendIntake(){
    leftIntakePiston.set(DoubleSolenoid.Value.kForward);
    rightIntakePiston.set(DoubleSolenoid.Value.kForward);
  }
  public void retractIntake(){
    leftIntakePiston.set(DoubleSolenoid.Value.kReverse);
    rightIntakePiston.set(DoubleSolenoid.Value.kReverse);
  }
  public void spinIn(double speed){
    rightIntakeSparkMax.set(speed);
    leftIntakeSparkMax.set(speed);
  }
  public void stopIntake(){
    rightIntakeSparkMax.set(0.0);
    leftIntakeSparkMax.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
