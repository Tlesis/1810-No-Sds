package frc.robot.subsystems;

import frc.robot.SwerveModule;

import static frc.robot.Constants.*;

import java.io.IOException;
import java.nio.file.Paths;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class DriveSubsystem {
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

    public void zeroGyro() {
        gyro.setFusedHeading(0);
    }

    public double getGyro() {
        return gyro.getFusedHeading();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getGyro());
    }

    public Pose2d getPose() {
        return m_odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometer.resetPosition(pose, getRotation2d());
    

    /************************ PAth STuff ************************/
    
    /**
     * Gets the path of a PathPlanner json file
     * @param trajectoryName the name of the PathPlanner path you want to call
     * @throws IOException
     */
    protected static Trajectory loadTrajectory(String trajectoryName) throws IOException {
        return TrajectoryUtil.fromPathweaverJson(
            Filesystem.getDeployDirectory().toPath().resolve(Paths.get("paths", trajectoryName + ".wpilib.json")));
    }
          
    /** 
     * Loads the PathPlanner File 
         * Loads the PathPlanner File 
     * Loads the PathPlanner File 
     * @param filename the name of the .json file
     */
    public Trajectory loadTrajectoryFromFile(String filename) {
        try {
            return loadTrajectory(filename);
        } catch (IOException e) {
            DriverStation.reportError("Failed to load auto trajectory: " + filename, false);
            return new Trajectory();
        }
    }
          
    /**
     * Creates a command to follow a Trajectory on the drivetrain.
     * @param trajectory trajectory to follow
     * @return command that will run the trajectory
     */
    public Command createCommandForTrajectory(Trajectory trajectory, Boolean initPose) {
    
        PIDController xController = new PIDController(AutoConstants.kp_X_CONTROLLER, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kp_Y_CONTROLLER, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kp_THETA_CONTROLLER, 0.0, 0.1, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            this::getPose,
            DriveConstants.DRIVE_KINEMATICS,
            xController,
            yController,
            thetaController,
            this::setModuleStates,
            this);
    
        if (initPose) {
            var reset =  new InstantCommand(() -> this.resetOdometry(trajectory.getInitialPose()));
            return reset.andThen(swerveControllerCommand.andThen(() -> stopModules()));
        } else {
            return swerveControllerCommand.andThen(() -> stopModules());
            
        }
    }
}