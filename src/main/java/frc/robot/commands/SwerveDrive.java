package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.*;

public class SwerveDrive extends CommandBase {
    private double ySpeed, xSpeed, rot;
    private boolean fieldRelative;
    private DriveSubsystem driveSubsystem;
    
    /**
     * Drive the robot using joystick info
     *
     * @param yInput Speed of the robot in the x direction
     * @param xInput Speed of the robot in the y direction
     * @param thetaInput rotation controller input (X)
     * @param fieldRelative Whether the bot should drive relative to the field
     */
    public SwerveDrive(double yInput, double xInput, double thetaInput,
                boolean fieldRelative, DriveSubsystem driveSubsystem) {
        this.ySpeed = yInput;
        this.xSpeed = xInput;
        this.rot = thetaInput;

        this.fieldRelative = fieldRelative;

        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        var swerveModuleStates =
            DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, driveSubsystem.getHeading())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
        driveSubsystem.frontLeft.setDesiredState(swerveModuleStates[0]);
        driveSubsystem.frontRight.setDesiredState(swerveModuleStates[1]);
        driveSubsystem.backLeft.setDesiredState(swerveModuleStates[2]);
        driveSubsystem.backRight.setDesiredState(swerveModuleStates[3]);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopModules();
    }
}