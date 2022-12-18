package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.PneumaticConstants;

public class ClawSubsystem extends SubsystemBase {
    private final WPI_TalonFX leftMotor, rightMotor;
    private final MotorControllerGroup motors;

    private final DoubleSolenoid clawPiston;

    private final DigitalInput cubeSensor;
    public ClawSubsystem() {
        leftMotor = new WPI_TalonFX(ShooterConstants.LEFT_MOTOR);
        rightMotor = new WPI_TalonFX(ShooterConstants.RIGHT_MOTOR);

        leftMotor.setInverted(false);
        rightMotor.follow(leftMotor);
        rightMotor.setInverted(InvertType.OpposeMaster);

        leftMotor.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
        rightMotor.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
        leftMotor.enableVoltageCompensation(true);
        rightMotor.enableVoltageCompensation(true);

        leftMotor.configSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIGURATION);
        leftMotor.configStatorCurrentLimit(ShooterConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);
        rightMotor.configSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIGURATION);
        rightMotor.configStatorCurrentLimit(ShooterConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);

        leftMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.setNeutralMode(NeutralMode.Coast);

        leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        leftMotor.config_kP(0, ShooterConstants.FLYWHEEL_KP);
        leftMotor.config_kI(0, ShooterConstants.FLYWHEEL_KI);
        leftMotor.config_kD(0, ShooterConstants.FLYWHEEL_KD);
        rightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        rightMotor.config_kP(0, ShooterConstants.FLYWHEEL_KP);
        rightMotor.config_kI(0, ShooterConstants.FLYWHEEL_KI);
        rightMotor.config_kD(0, ShooterConstants.FLYWHEEL_KD);

        leftMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
        leftMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        rightMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
        rightMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);

        motors = new MotorControllerGroup(leftMotor, rightMotor);

        clawPiston = new DoubleSolenoid(PneumaticConstants.deviceType, ShooterConstants.SOLENOID_CLAW[0], ShooterConstants.SOLENOID_CLAW[1]);

        cubeSensor = new DigitalInput(ShooterConstants.CUBE_SENSOR);
    }

    public void spinFlywheel(double power) {
        leftMotor.set(ControlMode.Velocity, );
    }


}

