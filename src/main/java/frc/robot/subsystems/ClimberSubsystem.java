package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final WPI_TalonFX climberMotor;
    public ClimberSubsystem() {
        climberMotor = new WPI_TalonFX(ClimberConstants.CLIMBER_MOTOR);

        climberMotor.setInverted(false);

        climberMotor.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
        climberMotor.enableVoltageCompensation(true);

        climberMotor.configSupplyCurrentLimit(Constants.ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIGURATION);
        climberMotor.configStatorCurrentLimit(Constants.ShooterConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);

        climberMotor.setNeutralMode(NeutralMode.Coast);

        climberMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        climberMotor.config_kP(0, Constants.ShooterConstants.FLYWHEEL_KP);
        climberMotor.config_kI(0, Constants.ShooterConstants.FLYWHEEL_KI);
        climberMotor.config_kD(0, Constants.ShooterConstants.FLYWHEEL_KD);

        climberMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
        climberMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

        climberMotor.configForwardSoftLimitThreshold(ClimberConstants.FORWARD_LIMIT);
        climberMotor.configReverseSoftLimitThreshold(ClimberConstants.REVERSE_LIMIT);
        climberMotor.configForwardSoftLimitEnable(true);
        climberMotor.configReverseSoftLimitEnable(true);
    }

    public void climbUp() {
        climberMotor.set(ClimberConstants.CLIMBER_SPEED);
    }

    public void climbDown() {
        climberMotor.set(-ClimberConstants.CLIMBER_SPEED);
    }

    public void stopClimb() {
        climberMotor.stopMotor();
    }
}

