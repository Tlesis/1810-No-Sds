package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * <p>
 * For example:
 * <pre> {@code import static frc.robot.Constants.*;} </pre>
 */
public class Constants {
    // https://drive.google.com/file/d/1g1jBZHPf6Fq6V2tG7PFIGtjpEEV2BIGf/view?usp=sharing
    public static final class DriveConstants {
        /**
         * The left-to-right distance between the drive wheels
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_TRACKWIDTH_METERS =  0.64928; // 0.5

        /**
         * The front-to-back distance between the drive wheels.
         *
         * Should be measured from center to center.
        */
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5; // 0.64928

        // Front left swerve module
        public static final int FRONT_LEFT_STEER_MOTOR_ID = 3;
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 4;
        public static final int FRONT_LEFT_CANCODER_ID = 9;
        public static final double FRONT_LEFT_STEER_OFFSET = 277.38;

        // Front right swerve module
        public static final int FRONT_RIGHT_STEER_MOTOR_ID = 5;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 6;
        public static final int FRONT_RIGHT_CANCODER_ID = 11;
        public static final double FRONT_RIGHT_STEER_OFFSET = 4.39;

        // Back left swerve module
        public static final int BACK_LEFT_STEER_MOTOR_ID = 1;
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 2;
        public static final int BACK_LEFT_CANCODER_ID = 10;
        public static final double BACK_LEFT_STEER_OFFSET = 240.03;

        // Back right swerve module
        public static final int BACK_RIGHT_STEER_MOTOR_ID = 7;
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 8;
        public static final int BACK_RIGHT_CANCODER_ID = 12;
        public static final double BACK_RIGHT_STEER_OFFSET = 132.54;

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
        );
    }

    /** pulled straight from SDS for the MK4 L2s */
    public static final class ModuleConstants {
        public static final double WHEEL_DIAMETER = 0.10033;
        public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        public static final boolean DRIVE_INVERTED = true;
        public static final double STEER_REDUCTION = (15.0 / 32.0) * (10.0 / 60.0);
        public static final boolean STEER_INVERTED = true;
        public static final boolean CANCODER_INVERTED = false;

        public static final double DRIVE_CURRENT_LIMIT = 80.0;
        public static final double STEER_CURRENT_LIMIT = 20.0;

        public static final double STEER_kP = 0.2; // 3087 = 0.6; SDS = 0.2;
        public static final double STEER_kI = 0.0;
        public static final double STEER_kD = 0.1; // 3087 = 12.0; SDS = 0.1;
        // increse to redude jitter
        public static final double ALLOWED_ERROR = 0; // (2048 * STEER_REDUCTION) / 360.0) = 1 degree

        public static final double DRIVE_kP = 0.1; // 3087 = 0.1; SDS = 0.0;
        public static final double DRIVE_kI = 0.0;
        public static final double DRIVE_kD = 0.0;

        // divide by 12 to convert from volts to % output for CTRE
        // pulled from 3087
        public static final double DRIVE_kS = (0.605 / 12);
        public static final double DRIVE_kV = (1.72 / 12);
        public static final double DRIVE_kA = (0.193 / 12);

        public static final double OPEN_LOOP_RAMP = 0.25; // From 3087
        public static final double CLOSED_LOOP_RAMP = 0;

        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Coast;
        public static final NeutralMode STEER_NEUTRAL_MODE = NeutralMode.Coast;

        /**
         * The maximum velocity of the robot in meters per second.
         * <p>
         * This is a measure of how fast the robot should be able to drive in a straight line.
         */
        // Freespeed: 4.96824 m/s
        // Calculated: 4.968230455 m/s
        // Real: (Not done) m/ (after whole robot is assembled)
        public static final double MAX_VELOCITY = 4.5;

        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
        // Freespeed: 61.21537703 rad/s
        // Calculated: 61.2152594258 rad/s
        // Real: (Not done) rad/s (after whole robot is assembled)
        public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY /
        Math.hypot(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0); // hypot is about 0.409745200826 meters
    }

    public static final class OIConstants {
        public static final int MOVEMENT_JOYSTICK_PORT = 0;
        public static final int ROTATION_JOYSTICK_PORT = 1;
        public static final int MANIPULATOR_CONTROLLER_PORT = 2;

        public static final double DEADBAND = 0.4;
    }

    public static final class GyroConstants {
        public static final int ID = 13;
        public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-
    }
}