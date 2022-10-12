package org.usd232.robotics.rapidreact.subsystems;

import org.usd232.robotics.rapidreact.log.Logger;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.wpilibj.XboxController.Axis;

// https://drive.google.com/file/d/1b3jYlQRw3vDhasCw-WSdM6FtWdolT9bM/view?usp=sharing

/** Used to turn the Xbox LT & RT from analog to Digital */
public class XboxTrigger extends Trigger {
    /**
     * The logger.
     * 
     * @since 2018
     */
    private static final Logger LOG = new Logger();
    
    private XboxController xbox;
    private final double minValue = 0.1;
    private Axis hand;

    public XboxTrigger(XboxController xbox, Axis hand) {
        this.xbox = xbox;
        this.hand = hand;
    }

    @Override
    public boolean get() {

        /* Debug Stuff */
        if (xbox.getRawAxis(hand.value) > minValue) {
            LOG.debug("Xbox %S returning TRUE", hand.toString());
        }

        return xbox.getRawAxis(hand.value) > minValue;
    }
    
}
