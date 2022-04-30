package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import frc.robot.Position;
import frc.robot.SwerveModule;
import frc.robot.math.SwerveCalc;

import static frc.robot.math.SwerveCalc.*;
import static java.lang.Double.max;

public class DriveSubsystem extends SubsystemBase {
    
    public final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
        DriveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
        DriveConstants.FRONT_LEFT_MODULE_STEER_CANCODER);

    public final SwerveModule backLeft = new SwerveModule(
        DriveConstants.BACK_LEFT_MODULE_STEER_MOTOR,
        DriveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
        DriveConstants.BACK_LEFT_MODULE_STEER_CANCODER);

    public final SwerveModule frontRight = new SwerveModule(
        DriveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
        DriveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
        DriveConstants.FRONT_RIGHT_MODULE_STEER_CANCODER);

    public final SwerveModule backRight = new SwerveModule(
        DriveConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
        DriveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
        DriveConstants.BACK_RIGHT_MODULE_STEER_CANCODER);

    public static PigeonIMU gyro;

    public DriveSubsystem() {
        gyro = new PigeonIMU(PigeonConstants.ID);
        gyro.configFactoryDefault();
    }

    public void zeroGyroscope() {
        gyro.setFusedHeading(0.0);
    }

    public void drive(double x, double y, double theta) throws Exception {

        new SwerveCalc(x, y, theta);

        double ratio = 1.0;

        double speedFL = getSpeed(x, y, theta, Position.FrontLeft);
        double speedBL = getSpeed(x, y, theta, Position.BackLeft);
        double speedFR = getSpeed(x, y, theta, Position.FrontRight);
        double speedBR = getSpeed(x, y, theta, Position.BackRight);

        double maxWheelSpeed = max(max(speedFL, speedBL), max(speedFR, speedBR));

        if (maxWheelSpeed > DriveConstants.MAX_SPEED) {
            ratio = (DriveConstants.MAX_SPEED / maxWheelSpeed);
        } else {
            ratio = 1.0;
        }

        frontLeft.set(ratio * speedFL, getAngle(x, y, theta, Position.FrontLeft));
        backLeft.set(ratio * speedBL, getAngle(x, y, theta, Position.BackLeft));
        frontRight.set(ratio * speedFR, getAngle(x, y, theta, Position.FrontRight));
        backRight.set(ratio * speedBR, getAngle(x, y, theta, Position.BackRight));
    }
}
