package org.usd232.robotics.rapidreact.commands;

import org.usd232.robotics.rapidreact.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Intake extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private final XboxController xbox;
    private final boolean right;
    
    public Intake(IntakeSubsystem intakeSubsystem, XboxController xbox, boolean right) {

        this.intakeSubsystem = intakeSubsystem;
        this.xbox = xbox;
        this.right = right;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() { /* I Was Here, so was I */ }

    @Override
    public void execute() {
        if (right) {
            intakeSubsystem.rightPneumatic(true);
            intakeSubsystem.rightMotor(!xbox.getBackButton());
        } else {
            intakeSubsystem.leftPneumatic(true);
            intakeSubsystem.leftMotor(!xbox.getBackButton());
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (right) {
            intakeSubsystem.rightPneumatic(false);
        } else {
            intakeSubsystem.leftPneumatic(false);
        }
        
        intakeSubsystem.motorOff(right);
    }
}
