package org.usd232.robotics.rapidreact.subsystems;

import static org.usd232.robotics.rapidreact.Constants.AugerConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AugerSubsystem extends SubsystemBase {
    
    private final static VictorSPX auger = new VictorSPX(VICTOR_ID);

    /** Turns the elevotor on */
    public void elevatorOn(boolean forward) {
        auger.set(ControlMode.PercentOutput, forward ? AUGER_ON : -AUGER_ON);
    }

    /** Turns the elevotor off */
    public void elevatorOff() {
        auger.set(ControlMode.PercentOutput, 0.0);
    }
}