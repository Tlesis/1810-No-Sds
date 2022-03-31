package frc.robot.commands.autonomous.paths;

import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OffLineReversed extends SequentialCommandGroup {
    public OffLineReversed(DriveSubsystem driveSubsystem) {
        Trajectory trajectory1 = driveSubsystem.loadTrajectoryFromFile("offLineReversed");

        addCommands(
            new InstantCommand(() -> {
                driveSubsystem.resetOdometry(trajectory1.getInitialPose());
            }),
            driveSubsystem.createCommandForTrajectory(trajectory1, false).withTimeout(15).withName("offLineReversed")
        );
    }
}