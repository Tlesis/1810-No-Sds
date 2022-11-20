package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.ModuleConstants.*;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveDrive extends CommandBase{

    private final DriveSubsystem driveSubsystem;
    private DoubleSupplier transX;
    private DoubleSupplier transY;
    private DoubleSupplier thetaSupplier;
    private final boolean fieldOriented;

    public SwerveDrive(DriveSubsystem driveSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               BooleanSupplier fieldOriented) {
        this.driveSubsystem = driveSubsystem;
        this.transX = translationXSupplier;
        this.transY = translationYSupplier;
        this.thetaSupplier = rotationSupplier;
        this.fieldOriented = fieldOriented.getAsBoolean();

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        Translation2d translation = new Translation2d(
                transX.getAsDouble(), transY.getAsDouble()).times(MAX_VELOCITY);

        driveSubsystem.drive(
            translation, 
            (thetaSupplier.getAsDouble() * MAX_ANGULAR_VELOCITY),
            fieldOriented,
            true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(new Translation2d(0, 0), 0, true, false);
    }
}
