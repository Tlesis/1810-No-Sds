// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        /**
         * The left-to-right distance between the drive wheels
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5;
        
        /**
         * The front-to-back distance between the drive wheels.
         *
         * Should be measured from center to center.
        */
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.64928;

        // Front left swerve module
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1;
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;
        public static final int FRONT_LEFT_MODULE_STEER_CANCODER = 10;
        
        // Front right swerve module
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3;
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4;
        public static final int FRONT_RIGHT_MODULE_STEER_CANCODER = 9;
        
        // Back left swerve module
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7;
        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 8;
        public static final int BACK_LEFT_MODULE_STEER_CANCODER = 12;
        
        // Back right swerve module
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5;
        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
        public static final int BACK_RIGHT_MODULE_STEER_CANCODER = 11;

        public static final  double X_SPEED_MAX = 3.2;
        public static final  double Y_SPEED_MAX = 3.2;
        public static final double THETA_SPEED_MAX = 6.0;

        public static final  double MAX_SPEED = 3.2;
    }

    public static final class ModuleConstants {

        public static final double STEER_kF = 0.0;
        public static final double STEER_kP = 0.07;
        public static final double STEER_kI = 0.0;
        public static final double STEER_kD = 0.3;

        public static double DRIVE_kF = 0.0;
        public static double DRIVE_kP = 0.07;
        public static double DRIVE_kI = 0.0;
        public static double DRIVE_kD = 0.6;

        public static final double STEER_RATIO = 12.8;
        public static final double DRIVE_RATIO = 6.86;

        public static final double WHEEL_RADIUS_METERS = 0.0508;
    }

    public static final class OIConstants {
        public static final int MOVEMENT_JOYSTICK_PORT = 0;
        public static final int ROTATION_JOYSTICK_PORT = 1;
        public static final int MANIPULATOR_CONTROLLER_PORT = 3;

        public static final double DEADBAND = 0.4;
    }

    public static final class PigeonConstants {
        public static final int ID = 13;
    }
}
