package frc.robot.swerve;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.Constants.*;

public class Gyro {
    public final PigeonIMU gyro;
    public Rotation2d yawOffset = new Rotation2d(0);

    /**
     * Creates a new Gyro, which is a wrapper for the Pigeon IMU and stores an offset so we don't
     * have to directly zero the gyro
     */
    public Gyro() {
        gyro = new PigeonIMU(GyroConstants.ID);
        gyro.configFactoryDefault();
        zeroGyro();
    }

    /** Zeros gyro */
    public void zeroGyro() {
        setGyroDegrees(0);
    }

    /**
     * Set the gyro yawOffset to a specific angle
     *
     * @param degrees The angle to set the gyro to
     */
    public void setGyroDegrees(double value) {
        yawOffset = getRawYaw().minus(new Rotation2d(value));
    }

    /**
     * Get the yaw of the robot in Rotation2d
     *
     * @return the yaw of the robot in Rotation2d
     */
    public Rotation2d getYaw() {
        return getRawYaw().minus(yawOffset);
    }

    /**
     * Get the raw yaw of the robot in Rotation2d without using the yawOffset
     *
     * @return the raw yaw of the robot in Rotation2d
     */
    public Rotation2d getRawYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    /**
     * Get the yaw of the robot in degrees
     *
     * @return the yaw of the robot in degrees including the yawOffset
     */
    public double getDegrees() {
        return getYaw().getDegrees();
    }

    /**
     * Get the raw yaw of the robot in Radians
     *
     * @return the yaw of the robot in radians including the yawOffset
     */
    public double getRadians() {
        return getYaw().getRadians();
    }
}
