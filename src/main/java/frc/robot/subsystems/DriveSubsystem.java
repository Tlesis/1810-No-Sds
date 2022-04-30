// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    // Robot swerve modules
    private final SwerveModule frontLeft =
        new SwerveModule(
            DriveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            DriveConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
            DriveConstants.FRONT_LEFT_MODULE_CANCODER,
            DriveConstants.FRONT_LEFT_MODULE_DRIVE_REVERSED,
            DriveConstants.FRONT_LEFT_MODULE_STEER_REVERSED,
            DriveConstants.FRONT_LEFT_MODULE_CANCODER_REVERSED);

    private final SwerveModule backLeft =
        new SwerveModule(
            DriveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
            DriveConstants.BACK_LEFT_MODULE_STEER_MOTOR,
            DriveConstants.BACK_LEFT_MODULE_CANCODER,
            DriveConstants.BACK_LEFT_MODULE_DRIVE_REVERSED,
            DriveConstants.BACK_LEFT_MODULE_STEER_REVERSED,
            DriveConstants.BACK_LEFT_MODULE_CANCODER_REVERSED);

    private final SwerveModule frontRight =
        new SwerveModule(
            DriveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            DriveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
            DriveConstants.FRONT_RIGHT_MODULE_CANCODER,
            DriveConstants.FRONT_RIGHT_MODULE_DRIVE_REVERSED,
            DriveConstants.FRONT_RIGHT_MODULE_STEER_REVERSED,
            DriveConstants.FRONT_RIGHT_MODULE_CANCODER_REVERSED);

    private final SwerveModule backRight =
        new SwerveModule(
            DriveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            DriveConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
            DriveConstants.BACK_RIGHT_MODULE_CANCODER,
            DriveConstants.BACK_RIGHT_MODULE_DRIVE_REVERSED,
            DriveConstants.BACK_RIGHT_MODULE_STEER_REVERSED,
            DriveConstants.BACK_RIGHT_MODULE_CANCODER_REVERSED);

    // The gyro sensor
    private final PigeonIMU gyro = new PigeonIMU(PigeonConstants.ID);

    // Odometry class for tracking robot pose
    SwerveDriveOdometry odometry =
        new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS, new Rotation2d(gyro.getFusedHeading()));

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {}

    @Override
    public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(
        new Rotation2d(gyro.getFusedHeading()),
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
    return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, new Rotation2d(gyro.getFusedHeading()));
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates =
            DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d(gyro.getFusedHeading()))
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /** Zeroes the heading of the robot. */
    public void zeroGyroscope() {
        gyro.setFusedHeading(0.0);
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return new Rotation2d(gyro.getFusedHeading()).getDegrees();
    }

    public void stopModules() {
        frontLeft.stop();
        backLeft.stop();
        frontRight.stop();
        backRight.stop();
    }
}
