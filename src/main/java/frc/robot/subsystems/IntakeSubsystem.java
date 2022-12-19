// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final DoubleSolenoid clawPistons;
  private final VictorSPX intakeMotor1;
  private final VictorSPX intakeMotor2;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    clawPistons = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.INTAKE_PISTONS_SOLENOID[0], IntakeConstants.INTAKE_PISTONS_SOLENOID[1]);
    intakeMotor1 = new VictorSPX(IntakeConstants.INTAKE_MOTOR_1);
    intakeMotor2 = new VictorSPX(IntakeConstants.INTAKE_MOTOR_2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void expand() {
    clawPistons.set(DoubleSolenoid.Value.kReverse);
  }

  public void retract() {
    clawPistons.set(DoubleSolenoid.Value.kForward);
  }

  public void runIntake() {
    intakeMotor1.set(ControlMode.PercentOutput, 0.5);
    intakeMotor2.set(ControlMode.PercentOutput, 0.5);
  }

  public void runIntake(double speed) {
    intakeMotor1.set(ControlMode.PercentOutput, speed);
    intakeMotor2.set(ControlMode.PercentOutput, speed);
  }

  public void outTake() {
    intakeMotor1.set(ControlMode.PercentOutput, -0.5);
    intakeMotor2.set(ControlMode.PercentOutput, -0.5);
  }

  public void outTake(double speed) {
    intakeMotor1.set(ControlMode.PercentOutput, -speed);
    intakeMotor2.set(ControlMode.PercentOutput, -speed);
  }

  public void stopIntake() {
    intakeMotor1.set(ControlMode.PercentOutput, 0);
    intakeMotor2.set(ControlMode.PercentOutput, 0);

  }

  public void togglePistons() {
    clawPistons.toggle();
  }

}
