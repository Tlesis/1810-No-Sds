// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * For example:
 * <pre> {@code import static frc.robot..Constants.*;} </pre>
 */
public final class Constants {
    public static final class DriveConstants {

        // Front left swerve module
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1;
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;
        public static final int FRONT_LEFT_MODULE_CANCODER = 10;
        public static final boolean FRONT_LEFT_MODULE_CANCODER_REVERSED = false; // FIXME
        public static final boolean FRONT_LEFT_MODULE_STEER_REVERSED = false; // FIXME
        public static final boolean FRONT_LEFT_MODULE_DRIVE_REVERSED = false; // FIXME
        
        // Front right swerve module
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3;
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4;
        public static final int FRONT_RIGHT_MODULE_CANCODER = 9;
        public static final boolean FRONT_RIGHT_MODULE_CANCODER_REVERSED = false; // FIXME
        public static final boolean FRONT_RIGHT_MODULE_STEER_REVERSED = false; // FIXME
        public static final boolean FRONT_RIGHT_MODULE_DRIVE_REVERSED = false; // FIXME
        
        // Back left swerve module
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7;
        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 8;
        public static final int BACK_LEFT_MODULE_CANCODER = 12;
        public static final boolean BACK_LEFT_MODULE_CANCODER_REVERSED = true; // FIXME
        public static final boolean BACK_LEFT_MODULE_STEER_REVERSED = true; // FIXME
        public static final boolean BACK_LEFT_MODULE_DRIVE_REVERSED = true; // FIXME
        
        // Back right swerve module
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5;
        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
        public static final int BACK_RIGHT_MODULE_CANCODER = 11;
        public static final boolean BACK_RIGHT_MODULE_CANCODER_REVERSED = true; // FIXME
        public static final boolean BACK_RIGHT_MODULE_STEER_REVERSED = true; // FIXME
        public static final boolean BACK_RIGHT_MODULE_DRIVE_REVERSED = true; // FIXME

        // Distance between centers of right and left wheels on robot
        public static final double TRACKWIDTH_METERS = 0.5;

        // Distance between front and back wheels on robot
        public static final double WHEELBASE_METERS = 0.64928;

        public static final SwerveDriveKinematics DRIVE_KINEMATICS =
            new SwerveDriveKinematics(
                new Translation2d(WHEELBASE_METERS / 2, TRACKWIDTH_METERS / 2),
                new Translation2d(WHEELBASE_METERS / 2, -TRACKWIDTH_METERS / 2),
                new Translation2d(-WHEELBASE_METERS / 2, TRACKWIDTH_METERS / 2),
                new Translation2d(-WHEELBASE_METERS / 2, -TRACKWIDTH_METERS / 2));

        public static final boolean GYRO_REVERSED = false;

        // TODO: These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The SysId tool provides a convenient method for obtaining these values for your robot.
        public static final double ksVolts = 1;
        public static final double kvVoltSecondsPerMeter = 0.8;
        public static final double kaVoltSecondsSquaredPerMeter = 0.15;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 2048.0 / 60.0 *
            ModuleConstants.DRIVE_REDUCTION * ModuleConstants.WHEEL_DIAMETER * Math.PI;
    }

    public static final class ModuleConstants {
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

        /** Drive reduction of SDS's MK4 Modules */
        public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

        /** Steer reduction of SDS's MK4 Modules */
        public static final double STEER_REDUCTION = (15.0 / 32.0) * (10.0 / 60.0);

        public static final int ENCODER_CPR = 2048;

        /** Wheel diameter in meters of SDS's MK4 Modules */
        public static final double WHEEL_DIAMETER = 0.10033;

        public static final double DRIVE_DISTANCE_PER_PULSE =
            DRIVE_REDUCTION * ((WHEEL_DIAMETER * Math.PI) / (double) ENCODER_CPR);

        public static final double STEER_DISTANCE_PER_PULSE =
            STEER_REDUCTION * ((2 * Math.PI) / (double) ENCODER_CPR);

        public static final double kP_STEER = 0.2;
        public static final double kD_STEER = 0.1;

        public static final double kP_DRIVE = 1; // TODO: FIXME
    }

    public static final class OIConstants {
        public static final int MOVEMENT_JOYSTICK_PORT = 0;
        public static final int ROTATION_JOYSTICK_PORT = 1;
        public static final int MANIPULATOR_CONTROLLER_PORT = 2;

        public static final double DEADBAND = 0.4;
    }

    /* public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    } */

    public static final class PigeonConstants {
        public static final int ID = 13;
    }
}
