package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.*;

public class Odometry {
    public SwerveDriveOdometry swerveOdometry;
    private DriveSubsystem driveSubsystem;

    public Odometry(DriveSubsystem s) {
        driveSubsystem = s;
        swerveOdometry =
            new SwerveDriveOdometry(
                DriveConstants.DRIVE_KINEMATICS,
                driveSubsystem.gyro.getYaw(),
                driveSubsystem.getPositions());
    }

    public SwerveDriveOdometry getSwerveDriveOdometry() {
        return swerveOdometry;
    }

    public void update() {
        swerveOdometry.update(driveSubsystem.gyro.getYaw(), driveSubsystem.getPositions());
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(driveSubsystem.gyro.getYaw(), driveSubsystem.getPositions(), pose);
    }

    public Pose2d getPoseMeters() {
        return swerveOdometry.getPoseMeters();
    }

    public double getXDistance() {
        return getPoseMeters().getX();
    }

    public double getYDistance() {
        return getPoseMeters().getY();
    }

    public double getDistance() {
        return getPoseMeters().getTranslation().getNorm();
    }
}
