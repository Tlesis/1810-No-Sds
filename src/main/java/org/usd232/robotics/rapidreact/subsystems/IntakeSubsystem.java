package org.usd232.robotics.rapidreact.subsystems;

import static org.usd232.robotics.rapidreact.Constants.IntakeConstants.*;

import org.usd232.robotics.rapidreact.Constants.PneumaticConstants;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Relay; // Motor controller is a spike
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    
    private static final Relay leftIntake = new Relay(LEFT_MOTOR_PORT, Relay.Direction.kBoth);
    private static final Relay rightIntake = new Relay(RIGHT_MOTOR_PORT, Relay.Direction.kBoth);

    private static final Solenoid leftSolenoid = new Solenoid(PneumaticConstants.PH_CAN_ID, PneumaticsModuleType.REVPH, LEFT_PNEUMATIC_PORT);
    private static final Solenoid rightSolenoid = new Solenoid(PneumaticConstants.PH_CAN_ID, PneumaticsModuleType.REVPH, RIGHT_PNEUMATIC_PORT);

    /** Lowers the left intake */
    public void leftPneumatic(boolean on) {
        leftSolenoid.set(on);
    }
    /** Lowers the right intake */
    public void rightPneumatic(boolean on) {
        rightSolenoid.set(on);
    }
    
    public void leftMotor(boolean forward) {
        if (!forward) {
            leftIntake.set(Relay.Value.kForward);
        } else {
            leftIntake.set(Relay.Value.kReverse);
        }
    }
    
    public void rightMotor(boolean forward) {
        if (!forward) {
            rightIntake.set(Relay.Value.kForward);
        } else {
            rightIntake.set(Relay.Value.kReverse);
        }

    }

    public void motorOff(boolean right) {
        if (right) {
            rightIntake.set(Relay.Value.kOff);
        } else {
            leftIntake.set(Relay.Value.kOff);
        }
    }
}
