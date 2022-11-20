package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.Gyro;
import frc.robot.swerve.Odometry;
import frc.robot.swerve.SwerveModule;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class DriveSubsystem extends SubsystemBase {
    public Gyro gyro;
    public Odometry odometry;
    public SwerveModule swerveModules[];
    private SwerveModuleState swerveModuleStates[];
    public double pidTurn = 0;
    public double driveX = 0;
    public double driveY = 0;
    public double driveTheta = 0;
    ChassisSpeeds lastRequestedVelocity = new ChassisSpeeds(0, 0, 0);

    public DriveSubsystem() {
        setName("Drive");
        gyro = new Gyro();

        SwerveModule frontLeft =
            new SwerveModule(0,
                DriveConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
                DriveConstants.FRONT_LEFT_STEER_MOTOR_ID,
                DriveConstants.FRONT_LEFT_CANCODER_ID,
                DriveConstants.FRONT_LEFT_STEER_OFFSET
            );

        SwerveModule frontRight =
            new SwerveModule(1,
                DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
                DriveConstants.FRONT_RIGHT_STEER_MOTOR_ID,
                DriveConstants.FRONT_RIGHT_CANCODER_ID,
                DriveConstants.FRONT_RIGHT_STEER_OFFSET
            );

        SwerveModule backLeft =
            new SwerveModule(2,
                DriveConstants.BACK_LEFT_DRIVE_MOTOR_ID,
                DriveConstants.BACK_LEFT_STEER_MOTOR_ID,
                DriveConstants.BACK_LEFT_CANCODER_ID,
                DriveConstants.BACK_LEFT_STEER_OFFSET
            );

        SwerveModule backRight =
            new SwerveModule(3,
                DriveConstants.BACK_RIGHT_DRIVE_MOTOR_ID,
                DriveConstants.BACK_RIGHT_STEER_MOTOR_ID,
                DriveConstants.BACK_RIGHT_CANCODER_ID,
                DriveConstants.BACK_RIGHT_STEER_OFFSET
            );

        swerveModules =
            new SwerveModule[] {
                frontLeft,
                frontRight,
                backLeft,
                backRight
            };

        resetSteeringToAbsolute();
        odometry = new Odometry(this);
    }

    public void drive(Translation2d translation, double theta, boolean fieldOriented, boolean isOpenLoop) {
        ChassisSpeeds speeds =
            (fieldOriented) ?
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        theta,
                        gyro.getYaw()) :

                new ChassisSpeeds(
                        translation.getX(),
                        translation.getY(),
                        theta);

        SwerveModuleState states[] = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, ModuleConstants.MAX_VELOCITY);

        for (var module : swerveModules) {
            module.setDesiredState(states[module.moduleNumber], isOpenLoop);
        }
    }

    // Reset AngleMotors to Absolute
    public void resetSteeringToAbsolute() {
        for (var module : swerveModules) {
            module.resetToAbsolute();
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.MAX_VELOCITY);

        for (var module : swerveModules) {
            module.setDesiredState(desiredStates[module.moduleNumber], false);
        }
    }

    public void brakeMode(boolean enabled) {
        for (var module : swerveModules) {
            if (enabled) {
                module.driveMotor.setNeutralMode(NeutralMode.Brake);
            } else {
                module.driveMotor.setNeutralMode(NeutralMode.Coast);
            }
        }
    }

    public void stop() {
        for (var module : swerveModules) {
            module.driveMotor.stopMotor();
            module.steerMotor.stopMotor();
        }
    }

    public SwerveModuleState[] getStates() {
        return swerveModuleStates;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (var module : swerveModules) {
            positions[module.moduleNumber] = module.getPosition();
        }
        return positions;
    }

    @Override
    public void periodic() {
        odometry.update();
        swerveModuleStates = getStatesCAN(); // Gets the states once a loop

        SmartDashboard.putString("SwerveModuleStates/Measured", swerveModuleStates.toString());
    }

    private SwerveModuleState[] getStatesCAN() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (var module : swerveModules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }
}
