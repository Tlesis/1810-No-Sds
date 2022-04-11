package frc.robot.commands.autonomous.paths;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OffLine extends SequentialCommandGroup {
    public OffLine(DriveSubsystem driveSubsystem) {
        PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("OffLine2", 3, 2);

        addCommands(
            new InstantCommand(() -> {
                driveSubsystem.resetOdometry(driveSubsystem.getInitPose(trajectory1));
            }),
            driveSubsystem.createCommandForTrajectory(trajectory1, true).withTimeout(15).withName("OffLine2")
        );
    }
}
