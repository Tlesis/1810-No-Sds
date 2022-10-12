package org.usd232.robotics.rapidreact.commands.autonomous.paths;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import org.usd232.robotics.rapidreact.subsystems.AugerSubsystem;
import org.usd232.robotics.rapidreact.subsystems.DriveSubsystem;
import org.usd232.robotics.rapidreact.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OfflineReversed extends SequentialCommandGroup {
    public OfflineReversed(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, AugerSubsystem augerSubsystem) {
        PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("offLineReversed", 3, 2);

        addCommands(
            new InstantCommand(() -> {
                driveSubsystem.resetOdometry(trajectory1.getInitialPose());
            }),
            new InstantCommand(() -> shooterSubsystem.shooterOn(0.435)),  
            new WaitCommand(2),
            new InstantCommand(() -> augerSubsystem.elevatorOn(true)),
            new WaitCommand(5),
            new InstantCommand(() -> augerSubsystem.elevatorOff()),
            driveSubsystem.createCommandForTrajectory(trajectory1, true).withTimeout(15).withName("offLineReversed")
        );
    }
}