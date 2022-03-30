package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveDrive extends CommandBase {
   
    private final DriveSubsystem driveSubsystem;
    private final Supplier<Double> xSpeed, ySpeed, thetaSpeed;
    private final boolean fieldOriented;
    private final SlewRateLimiter xLimiter, yLimiter, thetaLimiter;

    public SwerveDrive(DriveSubsystem driveSubsystem,
            Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> thetaSpeed,
            boolean fieldOriented) {
        this.driveSubsystem = driveSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.thetaSpeed = thetaSpeed;
        this.fieldOriented = fieldOriented;
        this.xLimiter = new SlewRateLimiter(DriveConstants.TELEOP_MAX_ACCELERATION_PER_SECOND);
        this.yLimiter = new SlewRateLimiter(DriveConstants.TELEOP_MAX_ACCELERATION_PER_SECOND);
        this.thetaLimiter = new SlewRateLimiter(DriveConstants.TELEOP_MAX_ANGULAR_ACCELERATION_PER_SECOND);
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = this.xSpeed.get();
        double ySpeed = this.ySpeed.get();
        double thetaSpeed = this.thetaSpeed.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.DEADBAND ? ySpeed : 0.0;
        thetaSpeed = Math.abs(thetaSpeed) > OIConstants.DEADBAND ? thetaSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.TELEOP_MAX_SPEED_PER_SEC;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.TELEOP_MAX_SPEED_PER_SEC;
        thetaSpeed = thetaLimiter.calculate(thetaSpeed)
                * DriveConstants.TELEOP_MAX_RADIANS_PER_SEC;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOriented) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, thetaSpeed, driveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        driveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
