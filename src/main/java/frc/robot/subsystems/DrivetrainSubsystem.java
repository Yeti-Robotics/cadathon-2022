package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.utils.controller.Controller;

public class DrivetrainSubsystem extends SubsystemBase {

    private final WPI_TalonFX leftMotor1, leftMotor2, rightMotor1, rightMotor2;
    private final MotorControllerGroup leftMotors, rightMotors;

    private final AHRS gyro;

    private final DifferentialDrive drive;
    private final DifferentialDriveWheelSpeeds wheelSpeeds;
    private final DifferentialDriveOdometry odometry;
    private DriveMode driveMode;

    public enum DriveMode {
        TANK,
        CHEEZY,
        ARCADE
    }

    private NeutralMode neutralMode;

    private final DoubleSolenoid shifterPiston;

    private final Controller controller;

    public DrivetrainSubsystem(Controller controller) {
        leftMotor1 = new WPI_TalonFX(DrivetrainConstants.LEFT_FALCON_1);
        leftMotor2 = new WPI_TalonFX(DrivetrainConstants.LEFT_FALCON_2);
        rightMotor1 = new WPI_TalonFX(DrivetrainConstants.RIGHT_FALCON_1);
        rightMotor2 = new WPI_TalonFX(DrivetrainConstants.RIGHT_FALCON_2);

        leftMotor1.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
        leftMotor2.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
        rightMotor1.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
        rightMotor2.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);

        leftMotor1.enableVoltageCompensation(true);
        leftMotor2.enableVoltageCompensation(true);
        rightMotor1.enableVoltageCompensation(true);
        rightMotor2.enableVoltageCompensation(true);

        leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);
        rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);
        rightMotors.setInverted(true);
        leftMotors.setInverted(true);
        setMotorsBrake();

        leftMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        rightMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        resetEncoders();

        gyro = new AHRS(Port.kUSB);
        resetGyro();

        drive = new DifferentialDrive(leftMotors, rightMotors);
        drive.setDeadband(0.0625);

        wheelSpeeds = new DifferentialDriveWheelSpeeds();

        odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

        shifterPiston = new DoubleSolenoid(Constants.PneumaticConstants.deviceType, DrivetrainConstants.SOLENOID_SHIFTER[0], DrivetrainConstants.SOLENOID_SHIFTER[1]);
        shiftUp();

        this.controller = controller;
    }

    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
    }

    public void setDriveMode(DriveMode mode) {
        driveMode = mode;
        switch (driveMode) {
            case TANK:
                this.setDefaultCommand(
                        new RunCommand(() -> tankDrive(controller.getLeftY(), controller.getRightY()), this));
                break;
            case CHEEZY:
                this.setDefaultCommand(
                        new RunCommand(() -> cheezyDrive(controller.getLeftY(), controller.getRightX()), this));
                break;
            case ARCADE:
                this.setDefaultCommand(
                        new RunCommand(() -> arcadeDrive(controller.getLeftY(), controller.getRightX()), this));
                break;
        }
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(rightVolts);
        drive.feed();
    }

    public void tankDrive(double leftpower, double rightpower) {
        drive.tankDrive(leftpower, rightpower);
    }

    public void cheezyDrive(double straight, double turn) {
        drive.curvatureDrive(straight, -turn, true);
    }

    public void arcadeDrive(double straight, double turn) {
        drive.arcadeDrive(straight, -turn);
    }

    public void stopDrive() {
        leftMotors.set(0.0);
        rightMotors.set(0.0);
    }

    public void setMotorsBrake() {
        leftMotor1.setNeutralMode(NeutralMode.Brake);
        leftMotor2.setNeutralMode(NeutralMode.Brake);
        rightMotor1.setNeutralMode(NeutralMode.Brake);
        rightMotor2.setNeutralMode(NeutralMode.Brake);
        neutralMode = NeutralMode.Brake;
    }

    public void setMotorsCoast() {
        leftMotor1.setNeutralMode(NeutralMode.Coast);
        leftMotor2.setNeutralMode(NeutralMode.Coast);
        rightMotor1.setNeutralMode(NeutralMode.Coast);
        rightMotor2.setNeutralMode(NeutralMode.Coast);
        neutralMode = NeutralMode.Coast;
    }

    // Distance in meters
    public double getLeftEncoderDistance() {
        return leftMotor1.getSelectedSensorPosition()
                * (getShifterPosition() == DoubleSolenoid.Value.kForward
                ? DrivetrainConstants.DISTANCE_PER_PULSE_HIGH_GEAR
                : DrivetrainConstants.DISTANCE_PER_PULSE_LOW_GEAR);
    }

    public double getRightEncoderDistance() {
        return -rightMotor1.getSelectedSensorPosition()
                * (getShifterPosition() == DoubleSolenoid.Value.kForward
                ? DrivetrainConstants.DISTANCE_PER_PULSE_HIGH_GEAR
                : DrivetrainConstants.DISTANCE_PER_PULSE_LOW_GEAR);
    }

    public double getAverageEncoder() {
        return ((getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0);
    }

    public void resetEncoders() {
        leftMotor1.setSelectedSensorPosition(0.0);
        rightMotor1.setSelectedSensorPosition(0.0);
    }

    // Velocity in meters/second
    public double getLeftEncoderVelocity() {
        return (leftMotor1.getSelectedSensorVelocity() * 10)
                * (getShifterPosition() == DoubleSolenoid.Value.kForward
                ? DrivetrainConstants.DISTANCE_PER_PULSE_HIGH_GEAR
                : DrivetrainConstants.DISTANCE_PER_PULSE_LOW_GEAR);
    }

    public double getRightEncoderVelocity() {
        return (-rightMotor1.getSelectedSensorVelocity() * 10)
                * (getShifterPosition() == DoubleSolenoid.Value.kForward
                ? DrivetrainConstants.DISTANCE_PER_PULSE_HIGH_GEAR
                : DrivetrainConstants.DISTANCE_PER_PULSE_LOW_GEAR);
    }

    public double getAverageVelocity() {
        return (getLeftEncoderVelocity() + getRightEncoderVelocity()) / 2.0;
    }

    public void toggleShifter() {
        shifterPiston.toggle();
    }

    public void shiftUp() {
        shifterPiston.set(DoubleSolenoid.Value.kForward);
    }

    public void shiftDown() {
        shifterPiston.set(DoubleSolenoid.Value.kReverse);
    }

    public DoubleSolenoid.Value getShifterPosition() {
        return shifterPiston.get();
    }
    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public void resetGyro() {
        gyro.reset();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        wheelSpeeds.leftMetersPerSecond = getLeftEncoderVelocity();
        wheelSpeeds.rightMetersPerSecond = getRightEncoderVelocity();
        return wheelSpeeds;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometer(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    public NeutralMode getNeutralMode() {
        return neutralMode;
    }
}