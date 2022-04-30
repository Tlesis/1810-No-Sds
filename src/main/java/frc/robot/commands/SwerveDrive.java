package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.math.SigmoidGenerator;
import frc.robot.subsystems.DriveSubsystem;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.StrictMath.PI;

public class SwerveDrive extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final Joystick rotConroller;
    private final Joystick moveController;
    private static final SigmoidGenerator responseCurve = new SigmoidGenerator(1.0);
    private static final SlewRateLimiter xLimiter = new SlewRateLimiter(4.5);
    private static final SlewRateLimiter yLimiter = new SlewRateLimiter(4.5);
    private static final SlewRateLimiter thetaLimiter = new SlewRateLimiter(4.5);
    private double angle;

    public SwerveDrive(DriveSubsystem driveSubsystem, Joystick moveController, Joystick rotConroller) {
        this.driveSubsystem = driveSubsystem;
        this.moveController = moveController;
        this.rotConroller = rotConroller;
        this.angle = DriveSubsystem.gyro.getFusedHeading();
        this.addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {
        driveSubsystem.frontLeft.zero();
        driveSubsystem.backLeft.zero();
        driveSubsystem.frontRight.zero();
        driveSubsystem.backRight.zero();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {

        angle = -(DriveSubsystem.gyro.getFusedHeading() % 360);

        double xInput = responseCurve.calculate(moveController.getX());
        double yInput = responseCurve.calculate(moveController.getY());

        try {
            driveSubsystem.drive(xLimiter.calculate(yInput * cos(angle / 360 * (2 * PI))) + xInput * sin(angle / 360 * (2 * PI)),
            yLimiter.calculate(yInput * sin(angle / 360 * (2 * PI))) - xInput * cos(angle /360 * (2 * PI)),
            thetaLimiter.calculate(rotConroller.getX()));
        } catch (Exception e) {}
    }
}
