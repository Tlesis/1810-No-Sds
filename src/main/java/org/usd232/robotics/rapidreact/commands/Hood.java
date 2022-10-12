package org.usd232.robotics.rapidreact.commands;

import static org.usd232.robotics.rapidreact.Constants.HoodConstants.*;
import org.usd232.robotics.rapidreact.subsystems.HoodSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Hood extends CommandBase {

    private final HoodSubsystem hoodSubsystem;
    private boolean forward;
    
    public Hood(HoodSubsystem hoodSubsystem, boolean forward) {
        this.forward = forward;
        this.hoodSubsystem = hoodSubsystem;
    }

    @Override
    public void execute() {
        if (forward) {
            hoodSubsystem.forwardHood();
        } else {
            hoodSubsystem.reverseHood();
        }
    }

    @Override
    public boolean isFinished() {
        if (HoodSubsystem.hoodEncoder.getDistance() <= FORWARD_HOOD_LIMIT && forward) {
            return true;
        }
        
        if (HoodSubsystem.hoodLS.get() && !forward) {
            hoodSubsystem.zeroEncoder();
            return true;
        }
        
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        hoodSubsystem.stopHood();
    }
}