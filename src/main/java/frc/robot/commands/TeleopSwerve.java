package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {
    private DriveSubsystem s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier fieldRelative;

    public TeleopSwerve(DriveSubsystem s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier fieldRelative) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.fieldRelative = fieldRelative;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), IOConstants.DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), IOConstants.DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), IOConstants.DEADBAND);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.DriveConstants.MAX_SPEED),
            rotationVal * Constants.DriveConstants.MAX_ANGULAR_VELOCITY,
            fieldRelative.getAsBoolean(),
            true
        );
    }
}