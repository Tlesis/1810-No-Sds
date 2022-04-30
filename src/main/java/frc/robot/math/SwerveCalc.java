package frc.robot.math;

import static frc.robot.Constants.*;
import frc.robot.Position;

import static java.lang.Math.atan2;

public class SwerveCalc {
    
    private static double alpha;
    private static double distToWheel;

    private static double xSpeed;
    private static double ySpeed;
    private static double thetaAddedSpeed;

    public SwerveCalc(double XInput, double yInput, double thetaInput) {
        //new Constants();

        alpha = Math.atan(DriveConstants.DRIVETRAIN_WHEELBASE_METERS / DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS);
        distToWheel = Math.sqrt((Math.pow(DriveConstants.DRIVETRAIN_WHEELBASE_METERS, 2)) +
                (Math.pow(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS, 2)));
        
        xSpeed = XInput * DriveConstants.X_SPEED_MAX;
        ySpeed = yInput * DriveConstants.Y_SPEED_MAX;
        thetaAddedSpeed = thetaInput * distToWheel * DriveConstants.THETA_SPEED_MAX;
    }

    public static void updateValues(double XInput, double yInput, double thetaInput) {
        xSpeed = XInput * DriveConstants.X_SPEED_MAX;
        ySpeed = yInput * DriveConstants.Y_SPEED_MAX;
        thetaAddedSpeed = thetaInput * DriveConstants.THETA_SPEED_MAX;
    }

    public static double getSpeed(double XInput, double yInput, double thetaInput, Position pos) throws Exception {
        double tempSpeedX;
        double tempSpeedY;

        updateValues(XInput, yInput, thetaInput);

        switch (pos) {
            case FrontLeft:
                tempSpeedX = xSpeed - thetaAddedSpeed * Math.cos(alpha);
                tempSpeedY = ySpeed - thetaAddedSpeed * Math.sin(alpha);
                return Math.sqrt(Math.pow(tempSpeedX, 2) + Math.pow(tempSpeedY, 2));

            case BackLeft:
                tempSpeedX = xSpeed - thetaAddedSpeed * Math.sin(alpha);
                tempSpeedY = ySpeed + thetaAddedSpeed * Math.cos(alpha);
                return Math.sqrt(Math.pow(tempSpeedX, 2) + Math.pow(tempSpeedY, 2));

            case FrontRight:
                tempSpeedX = xSpeed + thetaAddedSpeed * Math.sin(alpha);
                tempSpeedY = ySpeed - thetaAddedSpeed * Math.cos(alpha);
                return Math.sqrt(Math.pow(tempSpeedX, 2) + Math.pow(tempSpeedY, 2));

            case BackRight:
                tempSpeedX = xSpeed + thetaAddedSpeed * Math.cos(alpha);
                tempSpeedY = ySpeed + thetaAddedSpeed * Math.sin(alpha);
                return Math.sqrt(Math.pow(tempSpeedX, 2) + Math.pow(tempSpeedY, 2));

            default:
                throw new Exception("The speed was not initialized with a proper slot.");
        }
    }

    public static double getAngle(double xInput, double yInput, double thetaInput, Position pos) throws Exception {
        double tempSpeedX;
        double tempSpeedY;

        updateValues(xInput, yInput, thetaInput);
        
        switch (pos) {
            case FrontLeft:
                tempSpeedX = xSpeed - thetaAddedSpeed * Math.cos(alpha);
                tempSpeedY = ySpeed - thetaAddedSpeed * Math.sin(alpha);
                return atan2(tempSpeedX, tempSpeedY);

            case BackLeft:
                tempSpeedX = xSpeed - thetaAddedSpeed * Math.sin(alpha);
                tempSpeedY = ySpeed + thetaAddedSpeed * Math.cos(alpha);
                return atan2(tempSpeedX, tempSpeedY);

            case FrontRight:
                tempSpeedX = xSpeed + thetaAddedSpeed * Math.sin(alpha);
                tempSpeedY = ySpeed - thetaAddedSpeed * Math.cos(alpha);
                return atan2(tempSpeedX, tempSpeedY);

            case BackRight:
                tempSpeedX = xSpeed + thetaAddedSpeed * Math.cos(alpha);
                tempSpeedY = ySpeed + thetaAddedSpeed * Math.sin(alpha);
                return atan2(tempSpeedX, tempSpeedY);

            default:
                throw new Exception("The angle was not initialized with a proper slot.");
        }
    }
}
