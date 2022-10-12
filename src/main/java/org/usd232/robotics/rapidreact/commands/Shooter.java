package org.usd232.robotics.rapidreact.commands;

import org.usd232.robotics.rapidreact.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

// https://drive.google.com/file/d/1XJ0mosxRUbD-oGlpIFPcpEci18Gh_oIK

public class Shooter extends CommandBase {

    private final ShooterSubsystem shooterSubsystem;
    private final double percent;
    
    public Shooter(ShooterSubsystem shooterSubsystem, double percent) {
        this.shooterSubsystem = shooterSubsystem;
        this.percent = percent;
        addRequirements(shooterSubsystem);
    }


    @Override
    public void execute() {
        shooterSubsystem.shooterOn(percent);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.shooterOff();
    }
}
