package org.usd232.robotics.rapidreact.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import static edu.wpi.first.wpilibj.XboxController.Axis;

// https://drive.google.com/file/d/1b3jYlQRw3vDhasCw-WSdM6FtWdolT9bM/view?usp=sharing

/** Used to turn the Xbox LT & RT from analog to Digital */
public class XboxTrigger extends Button {
    
    private XboxController xbox;
    private static final double minValue = 0.1;
    private Axis hand;

    public XboxTrigger(XboxController xbox, Axis hand) {
        this.xbox = xbox;
        this.hand = hand;
    }

    @Override
    public boolean get() {
        return xbox.getRawAxis(hand.value) > minValue;
    }
    
}
