package org.usd232.robotics.rapidreact.commands;

import org.usd232.robotics.rapidreact.subsystems.AugerSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Auger extends CommandBase {

    private final AugerSubsystem augerSubsystem;
    private final XboxController xbox;
    
    /** Command that controls the, the auger  */
    public Auger(AugerSubsystem augerSubsystem, XboxController xbox) {
        this.augerSubsystem = augerSubsystem;
        this.xbox = xbox;
    }

    @Override
    public void execute() {
        augerSubsystem.elevatorOn(!xbox.getBackButton());
    }

    @Override
    public void end(boolean interrupted) {
        augerSubsystem.elevatorOff();
    }
}
