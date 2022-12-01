package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.SwerveModule;
import frc.robot.Constants.GyroConstants;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] modules;
    public PigeonIMU gyro;

    public DriveSubsystem() {
        gyro = new PigeonIMU(Constants.GyroConstants.ID);
        gyro.configFactoryDefault();
        zeroGyro();

        swerveOdometry = new SwerveDriveOdometry(Constants.DriveConstants.SWERVE_KINEMATICS, getYaw());

        modules = new SwerveModule[] {
            new SwerveModule(0, Constants.DriveConstants.FLMod.constants),
            new SwerveModule(1, Constants.DriveConstants.FRMod.constants),
            new SwerveModule(2, Constants.DriveConstants.BLMod.constants),
            new SwerveModule(3, Constants.DriveConstants.BRMod.constants)
        };
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(),
                                    translation.getY(),
                                    rotation,
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(),
                                    translation.getY(),
                                    rotation)
                                );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveConstants.MAX_SPEED);

        for(SwerveModule mod : modules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.MAX_SPEED);

        for(SwerveModule mod : modules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, getYaw());
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : modules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        double ypr[] = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (GyroConstants.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getStates());

        for(SwerveModule mod : modules) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}