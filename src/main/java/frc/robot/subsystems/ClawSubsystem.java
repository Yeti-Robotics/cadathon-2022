package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.PneumaticConstants;

public class ClawSubsystem extends SubsystemBase {
    private final WPI_TalonFX leftMotor, rightMotor, pivotMotor;
    private final SimpleMotorFeedforward flywheelFF;
    private final PIDController flywheelPID;

    public enum ClawPositions {
        START,
        SHOOTING,
        INTAKING
    }
    private ClawPositions clawPosition;

    private final DoubleSolenoid clawPiston;

    private final DigitalInput cubeSensor;

    private double setPoint = 0.0;
    private boolean isSpinning = false;
    public ClawSubsystem() {
        leftMotor = new WPI_TalonFX(ShooterConstants.LEFT_MOTOR);
        rightMotor = new WPI_TalonFX(ShooterConstants.RIGHT_MOTOR);
        pivotMotor = new WPI_TalonFX(ShooterConstants.PIVOT_MOTOR);

        leftMotor.setInverted(false);
        rightMotor.follow(leftMotor);
        rightMotor.setInverted(InvertType.OpposeMaster);

        leftMotor.configSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIGURATION);
        leftMotor.configStatorCurrentLimit(ShooterConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);
        rightMotor.configSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIGURATION);
        rightMotor.configStatorCurrentLimit(ShooterConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);
        pivotMotor.configSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT_CONFIGURATION);
        pivotMotor.configStatorCurrentLimit(ShooterConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);

        leftMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.setNeutralMode(NeutralMode.Coast);
        pivotMotor.setNeutralMode(NeutralMode.Brake);

        leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        rightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

        leftMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
        leftMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        rightMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
        rightMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);
        pivotMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
        pivotMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);


        pivotMotor.configForwardSoftLimitThreshold(ShooterConstants.FORWARD_LIMIT);
        pivotMotor.configReverseSoftLimitThreshold(ShooterConstants.REVERSE_LIMIT);
        pivotMotor.configForwardSoftLimitEnable(true);
        pivotMotor.configReverseSoftLimitEnable(true);

        pivotMotor.configMotionCruiseVelocity(ShooterConstants.PIVOT_MAX_VEL);
        pivotMotor.configMotionSCurveStrength(ShooterConstants.PIVOT_ACCEL_SMOOTHING);
        pivotMotor.configMotionAcceleration(ShooterConstants.PIVOT_MAX_ACCEL);
        pivotMotor.config_kF(0, ShooterConstants.PIVOT_KF);
        pivotMotor.config_kP(0, ShooterConstants.PIVOT_KP);
        pivotMotor.config_kI(0, ShooterConstants.PIVOT_KI);
        pivotMotor.config_kD(0, ShooterConstants.PIVOT_KD);

        flywheelFF = new SimpleMotorFeedforward(
                ShooterConstants.FLYWHEEL_KS, ShooterConstants.FLYWHEEL_KV, ShooterConstants.FLYWHEEL_KA);
        flywheelPID = new PIDController(
                ShooterConstants.FLYWHEEL_KP, ShooterConstants.FLYWHEEL_KI, ShooterConstants.FLYWHEEL_KD);

        clawPiston = new DoubleSolenoid(
                PneumaticConstants.deviceType, ShooterConstants.SOLENOID_CLAW[0], ShooterConstants.SOLENOID_CLAW[1]);

        cubeSensor = new DigitalInput(ShooterConstants.CUBE_SENSOR);

        closeClaw();
        clawPosition = ClawPositions.START;
    }

    @Override
    public void periodic() {
        if (isSpinning) {
            double feedForward = flywheelFF.calculate(setPoint, ShooterConstants.FLYWHEEL_ACCEL);
            setFlywheelVolts(flywheelPID.calculate(getFlywheelVelocity(), setPoint + feedForward));
        }
    }

    public void setFlywheelSetpoint(double speed) {
        isSpinning = true;
        setPoint = Math.min(speed, ShooterConstants.MAX_SPEED);
    }

    private void setFlywheelVolts(double volts) {
        leftMotor.setVoltage(volts);
    }

    public void stopFlywheel() {
        isSpinning = false;
        leftMotor.stopMotor();
    }

    public boolean getIsFlywheelSpinning() {
        return isSpinning;
    }

    public double getFlywheelVelocity() {
        return leftMotor.getSelectedSensorVelocity() * 10 * ShooterConstants.METER_PER_PULSE;
    }

    public boolean isCubeInClaw() {
        return cubeSensor.get();
    }

    public void toggleClaw() {
        clawPiston.toggle();
    }

    public void closeClaw() {
        clawPiston.set(DoubleSolenoid.Value.kReverse);
    }

    public void openClaw() {
        clawPiston.set(DoubleSolenoid.Value.kForward);
    }

    public DoubleSolenoid.Value getClawPosition() {
        return clawPiston.get();
    }

    public void toggleClawPosition() {
        if (clawPosition != ClawPositions.INTAKING) {
            intakePositionClaw();
        } else {
            shootingPositionClaw();
        }
    }

    public void startingPositionClaw() {
        clawPosition = ClawPositions.START;
        pivotMotor.set(ControlMode.MotionMagic, ShooterConstants.STARTING_POSITION);
    }

    public void shootingPositionClaw() {
        clawPosition = ClawPositions.SHOOTING;
        pivotMotor.set(ControlMode.MotionMagic, ShooterConstants.SHOOTING_POSITION);
    }

    public void intakePositionClaw() {
        clawPosition = ClawPositions.INTAKING;
        pivotMotor.set(ControlMode.MotionMagic, ShooterConstants.INTAKE_POSITION);
    }
}

