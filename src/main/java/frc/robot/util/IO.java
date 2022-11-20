package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.log.Logger;

/**
 * Class that "hides" the button bindings<p>
 * Import only where needed with
 * <pre>{@code import static frc.robot.IO.*;}</pre>
 */
public final class IO {
    /**
     * The logger.
     *
     * @since 2018
     */
    //@SuppressWarnings("unused")
    private static final Logger LOG = new Logger();

    public static final Joystick movementJoystick = LOG.catchAll(() -> new Joystick(OIConstants.MOVEMENT_JOYSTICK_PORT));
    public static final Joystick rotationJoystick = LOG.catchAll(() -> new Joystick(OIConstants.ROTATION_JOYSTICK_PORT));
    public static final XboxController manipulatorController = LOG.catchAll(() -> new XboxController(OIConstants.MANIPULATOR_CONTROLLER_PORT));

    // Xbox buttons
    public static final JoystickButton ManipulatorXbox_B = LOG.catchAll(() -> new JoystickButton(manipulatorController, 2));
    public static final JoystickButton ManipulatorXbox_X = LOG.catchAll(() -> new JoystickButton(manipulatorController, 3));
    public static final JoystickButton ManipulatorXbox_A = LOG.catchAll(() -> new JoystickButton(manipulatorController, 1));
    public static final JoystickButton ManipulatorXbox_Y = LOG.catchAll(() -> new JoystickButton(manipulatorController, 4));
    public static final JoystickButton ManipulatorXbox_LB = LOG.catchAll(() -> new JoystickButton(manipulatorController, 5));
    public static final JoystickButton ManipulatorXbox_RB = LOG.catchAll(() -> new JoystickButton(manipulatorController, 6));
    public static final JoystickButton ManipulatorXbox_Back = LOG.catchAll(() -> new JoystickButton(manipulatorController, 7));
    public static final JoystickButton ManipulatorXbox_Start = LOG.catchAll(() -> new JoystickButton(manipulatorController, 8));
    public static final JoystickButton ManipulatorXbox_LStick = LOG.catchAll(() -> new JoystickButton(manipulatorController, 9));
    public static final JoystickButton ManipulatorXbox_RStick = LOG.catchAll(() -> new JoystickButton(manipulatorController, 10));

    // Joystick Buttons
    public static final JoystickButton movementJoystick_Trigger = LOG.catchAll(() -> new JoystickButton(movementJoystick, 1));
    public static final JoystickButton movementJoystick_Button2 = LOG.catchAll(() -> new JoystickButton(movementJoystick, 2));
    public static final JoystickButton movementJoystick_Button3 = LOG.catchAll(() -> new JoystickButton(movementJoystick, 3));
    public static final JoystickButton movementJoystick_Button4 = LOG.catchAll(() -> new JoystickButton(movementJoystick, 4));
    public static final JoystickButton movementJoystick_Button5 = LOG.catchAll(() -> new JoystickButton(movementJoystick, 5));
    public static final JoystickButton movementJoystick_Button6 = LOG.catchAll(() -> new JoystickButton(movementJoystick, 6));
    public static final JoystickButton movementJoystick_Button7 = LOG.catchAll(() -> new JoystickButton(movementJoystick, 7));
    public static final JoystickButton movementJoystick_Button8 = LOG.catchAll(() -> new JoystickButton(movementJoystick, 8));
    public static final JoystickButton movementJoystick_Button9 = LOG.catchAll(() -> new JoystickButton(movementJoystick, 9));
    public static final JoystickButton movementJoystick_Button10 = LOG.catchAll(() -> new JoystickButton(movementJoystick, 10));
    public static final JoystickButton movementJoystick_Button11 = LOG.catchAll(() -> new JoystickButton(movementJoystick, 11));

    // Joystick Buttons
    public static final JoystickButton rotationJoystick_Trigger = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 1));
    public static final JoystickButton rotationJoystick_Button2 = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 2));
    public static final JoystickButton rotationJoystick_Button3 = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 3));
    public static final JoystickButton rotationJoystick_Button4 = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 4));
    public static final JoystickButton rotationJoystick_Button5 = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 5));
    public static final JoystickButton rotationJoystick_Button6 = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 6));
    public static final JoystickButton rotationJoystick_Button7 = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 7));
    public static final JoystickButton rotationJoystick_Button8 = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 8));
    public static final JoystickButton rotationJoystick_Button9 = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 9));
    public static final JoystickButton rotationJoystick_Button10 = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 10));
    public static final JoystickButton rotationJoystick_Button11 = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 11));

    private IO() {/* what does sleep feel like */}
}