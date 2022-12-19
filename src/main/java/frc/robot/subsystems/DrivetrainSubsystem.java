package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    private final WPI_TalonFX leftTalon1;
    private final WPI_TalonFX leftTalon2;
    private final WPI_TalonFX rightTalon1;
    private final WPI_TalonFX rightTalon2;

    private final MotorControllerGroup left;

    private final MotorControllerGroup right;

    private final WPI_Pigeon2 gyro;

    private final DifferentialDrive differentialDrive;

    private final DifferentialDriveOdometry differentialDriveOdometry;
    public DrivetrainSubsystem() {
        leftTalon1 = new WPI_TalonFX(Constants.DrivetrainConstants.LEFT_TALON_1);

        leftTalon2 = new WPI_TalonFX(Constants.DrivetrainConstants.LEFT_TALON_2);

        rightTalon1 = new WPI_TalonFX(Constants.DrivetrainConstants.RIGHT_TALON_1);

        rightTalon2 = new WPI_TalonFX(Constants.DrivetrainConstants.RIGHT_TALON_2);

        left = new MotorControllerGroup(leftTalon1, leftTalon2);

        right = new MotorControllerGroup(rightTalon1, rightTalon2);

        gyro = new WPI_Pigeon2(Constants.DrivetrainConstants.GYRO);

        differentialDrive = new DifferentialDrive(left, right);

        differentialDriveOdometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    }
    public void cheesyDrive(double straight, double turn){
        differentialDrive.curvatureDrive(straight, turn, false);
    }
}