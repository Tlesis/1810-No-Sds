package org.usd232.robotics.rapidreact.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;

// import org.usd232.robotics.rapidreact.log.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.usd232.robotics.rapidreact.Constants.AutoConstants;
import static org.usd232.robotics.rapidreact.Constants.DriveConstants;
import static org.usd232.robotics.rapidreact.Constants.ModuleConstants;
import static org.usd232.robotics.rapidreact.Constants.PigeonConstants;

import org.usd232.robotics.rapidreact.SwerveModule;

// https://drive.google.com/file/d/1jjWRu1KV4cwF8fZrVr89JhTrMbSx8Aeh/view?usp=sharing

public class DriveSubsystem extends SubsystemBase {
    /**
     * The logger.
     * 
     * @since 2018
     */
    //@SuppressWarnings("unused")
    // private static final Logger LOG = new Logger();

    private final static PigeonIMU m_pigeon = new PigeonIMU(PigeonConstants.ID);

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private final SwerveDriveOdometry m_odometer;

    /** Anytime an object of the DriveSubsystem class is called, this constructor sets up the swerve modules */
    public DriveSubsystem() {
        ShuffleboardTab driveTrainTab = Shuffleboard.getTab("Drivetrain");
// Mom loves you
        frontLeft = new SwerveModule(
                    driveTrainTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                    // This is the ID of the drive motor
                    DriveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                    // This is the ID of the steer motor
                    DriveConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
                    // This is the ID of the steer encoder
                    DriveConstants.FRONT_LEFT_MODULE_STEER_CANCODER,
                    // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
                    DriveConstants.FRONT_LEFT_MODULE_STEER_OFFSET);

        frontRight = new SwerveModule(
                    driveTrainTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                    DriveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                    DriveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                    DriveConstants.FRONT_RIGHT_MODULE_STEER_CANCODER,
                    DriveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);

        backLeft = new SwerveModule(
                    driveTrainTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                    DriveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                    DriveConstants.BACK_LEFT_MODULE_STEER_MOTOR,
                    DriveConstants.BACK_LEFT_MODULE_STEER_CANCODER,
                    DriveConstants.BACK_LEFT_MODULE_STEER_OFFSET);

        backRight = new SwerveModule(
                    driveTrainTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                    DriveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                    DriveConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
                    DriveConstants.BACK_RIGHT_MODULE_STEER_CANCODER,
                    DriveConstants.BACK_RIGHT_MODULE_STEER_OFFSET);

        m_odometer = new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS,
            new Rotation2d(0));
    }

    /**
     * Sets the gyroscope angle to zero.
     * This can be used to set the direction
     * the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        m_pigeon.setFusedHeading(0.0);
    }

    /** Grabs the rotation of the gyroscope */
    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
    }

    /** @return the gyro angle as a double */
    public static double getGyro() {
        return m_pigeon.getFusedHeading();
    }

    public static boolean ifGyroZero() {
        return (getGyro() > -1 || getGyro() < 1);
    }

    /** 
     * @param modulePosition which module you are referencing ({@link DriveSubsystem#Module})
     * @return current module state as a {@link SwerveModuleState} 
     */
    public SwerveModuleState getState(SwerveModule module) {
        return new SwerveModuleState(module.getDriveVelocity(), new Rotation2d(module.getAbsoluteAngle()));
    }

    /** @return the Pigeon's Heading as a Rotation2d object
     * <p>
     * Used for WPI stuff usually
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getGyro());
    }

    /** @return a Pose2d object of the current pose mesured by the odometer 
     * <p>
     * Used for WPI stuff usually
     */
    public Pose2d getPose() {
        return m_odometer.getPoseMeters();
    }

    /** resets the Odometer to current position
     * @param pose Pose2d Object to reset to current position
     */
    public void resetOdometry(Pose2d pose) {
        m_odometer.resetPosition(pose, getRotation2d());
    }

    /** Very important function. Takes in the chassis speeds, then keeps them for later handling */
    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    public void setModuleStates(SwerveModuleState states[]) {
        // reduce ice driving?
        double statesAvg = 0;
        for (int i = 0; i < 4; i++) {
            statesAvg += Math.abs(states[i].speedMetersPerSecond);
        }
        if (statesAvg < 1) { // TODO: test < value
            setNeutralMode(NeutralMode.Brake);
        } else {
            setNeutralMode(NeutralMode.Coast);
        }

        // Normalizes the wheel speeds, scaling the speed of all wheels down relative to each other. This just means that no one wheel will be going too fast when it shouldn't
        SwerveDriveKinematics.desaturateWheelSpeeds(states, ModuleConstants.MAX_VELOCITY_METERS_PER_SECOND);
        // Set the motor speeds and rotation
        frontLeft.set(states[0].speedMetersPerSecond / ModuleConstants.MAX_VELOCITY_METERS_PER_SECOND * ModuleConstants.MAX_VOLTAGE, states[0].angle.getRadians());
        frontRight.set(states[1].speedMetersPerSecond / ModuleConstants.MAX_VELOCITY_METERS_PER_SECOND * ModuleConstants.MAX_VOLTAGE, states[1].angle.getRadians());
        backLeft.set(states[2].speedMetersPerSecond / ModuleConstants.MAX_VELOCITY_METERS_PER_SECOND * ModuleConstants.MAX_VOLTAGE, states[2].angle.getRadians());
        backRight.set(states[3].speedMetersPerSecond / ModuleConstants.MAX_VELOCITY_METERS_PER_SECOND * ModuleConstants.MAX_VOLTAGE, states[3].angle.getRadians());
    }

    /** Periodically does stuff */
    @Override
    public void periodic() {
        m_odometer.update(getRotation2d(), getState(frontLeft), getState(frontRight),
                            getState(backLeft), getState(backRight));

        SwerveModuleState[] states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(m_chassisSpeeds);
        setModuleStates(states);

        SmartDashboard.putString("Odometry Pose", getPose().toString());
    }

    /** Stops Robot */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setNeutralMode(NeutralMode mode) {
        frontLeft.setDriveNeutralMode(mode);
        frontRight.setDriveNeutralMode(mode);
        backLeft.setDriveNeutralMode(mode);
        backRight.setDriveNeutralMode(mode);
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
