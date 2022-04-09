package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import frc.robot.SwerveModule;
import io.github.oblarg.oblog.annotations.Log;

public class DriveSubsystem extends SubsystemBase {
// Mom loves you
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR, 
        DriveConstants.FRONT_LEFT_MODULE_STEER_MOTOR, 
        DriveConstants.FRONT_LEFT_MODULE_STEER_CANCODER,
        DriveConstants.FRONT_LEFT_MODULE_STEER_OFFSET, 
        DriveConstants.FRONT_LEFT_DRIVE_REVERSED, 
        DriveConstants.FRONT_LEFT_STEER_REVERSED, 
        DriveConstants.FRONT_LEFT_CANCODER_REVERSED, 
        DriveConstants.FRONT_LEFT);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR, 
        DriveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR, 
        DriveConstants.FRONT_RIGHT_MODULE_STEER_CANCODER, 
        DriveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET, 
        DriveConstants.FRONT_RIGHT_DRIVE_REVERSED, 
        DriveConstants.FRONT_RIGHT_STEER_REVERSED, 
        DriveConstants.FRONT_RIGHT_CANCODER_REVERSED, 
        DriveConstants.FRONT_RIGHT);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR, 
        DriveConstants.BACK_LEFT_MODULE_STEER_MOTOR, 
        DriveConstants.BACK_LEFT_MODULE_STEER_CANCODER, 
        DriveConstants.BACK_LEFT_MODULE_STEER_OFFSET, 
        DriveConstants.BACK_LEFT_DRIVE_REVERSED, 
        DriveConstants.BACK_LEFT_STEER_REVERSED, 
        DriveConstants.BACK_LEFT_CANCODER_REVERSED, 
        DriveConstants.BACK_LEFT);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR, 
        DriveConstants.BACK_RIGHT_MODULE_STEER_MOTOR, 
        DriveConstants.BACK_RIGHT_MODULE_STEER_CANCODER, 
        DriveConstants.BACK_RIGHT_MODULE_STEER_OFFSET, 
        DriveConstants.BACK_RIGHT_DRIVE_REVERSED, 
        DriveConstants.BACK_RIGHT_STEER_REVERSED, 
        DriveConstants.BACK_RIGHT_CANCODER_REVERSED, 
        DriveConstants.BACK_RIGHT);

    private final PigeonIMU gyro = new PigeonIMU(PigeonConstants.ID);
    private final SwerveDriveOdometry m_odometer = new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS, new Rotation2d(0));

    public void zeroGyroscope() {
        gyro.setFusedHeading(0);
    }

    @Log(name = "Gyroscope angle")
    public double getGyro() {
        return gyro.getFusedHeading();
    }

    @Log(name = "Gyro 0")
    public boolean ifGyroZero() {
        return (getGyro() < 1 || getGyro() > -1);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getGyro());
    }

    public Pose2d getPose() {
        return m_odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometer.resetPosition(pose, getRotation2d());
    }

    @Override
    public void periodic() {
        m_odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(),
                          backLeft.getState(), backRight.getState());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState desiredStates[]) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /************************ PAth STuff ************************/
    
    public Pose2d getInitPose(PathPlannerTrajectory trajectory) {
        return new Pose2d(trajectory.getInitialState().poseMeters.getTranslation(),
            trajectory.getInitialState().holonomicRotation);
    }
          
    /**
     * Creates a command to follow a Trajectory on the drivetrain.
     * @param trajectory trajectory to follow
     * @return command that will run the trajectory
     */
    public Command createCommandForTrajectory(PathPlannerTrajectory trajectory, Boolean initPose) {
    
        PIDController xController = new PIDController(AutoConstants.kp_X_CONTROLLER, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kp_Y_CONTROLLER, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kp_THETA_CONTROLLER, 0.0, 0.1, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
            trajectory,
            this::getPose,
            DriveConstants.DRIVE_KINEMATICS,
            xController,
            yController,
            thetaController,
            this::setModuleStates,
            this);
    
        if (initPose) {
            var reset =  new InstantCommand(() -> this.resetOdometry(getInitPose(trajectory)));
            return reset.andThen(swerveControllerCommand.andThen(() -> stopModules()));
        } else {
            return swerveControllerCommand.andThen(() -> stopModules());
        }
    }

}
