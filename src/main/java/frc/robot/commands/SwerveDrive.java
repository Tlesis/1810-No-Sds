package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveDrive extends CommandBase {
    private double yInput, xInput, thetaInput;
    private boolean fieldRelative;
    private DriveSubsystem driveSubsystem;
    
    public SwerveDrive(double yInput, double xInput, double thetaInput,
                boolean fieldRelative, DriveSubsystem driveSubsystem) {
        this.yInput = yInput;
        this.xInput = xInput;
        this.thetaInput = thetaInput;

        this.fieldRelative = fieldRelative;

        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.drive(
            yInput,
            xInput,
            thetaInput,
            fieldRelative);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopModules();
    }
}