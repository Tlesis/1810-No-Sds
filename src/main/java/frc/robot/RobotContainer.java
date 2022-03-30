package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveDrive;
import frc.robot.log.Logger;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /**
     * The logger.
     * 
     * @since 2018
     */
    private static final Logger LOG = new Logger();

    // private SendableChooser<Command> pathChooser = new SendableChooser<>();

    private final DriveSubsystem m_driveSubsystem = LOG.catchAll(() -> new DriveSubsystem());

    /* Contollers */
    private final Joystick movementJoystick = LOG.catchAll(() -> new Joystick(OIConstants.MOVEMENT_JOYSTICK_PORT));
    private final Joystick rotationJoystick = LOG.catchAll(() -> new Joystick(OIConstants.ROTATION_JOYSTICK_PORT));
    // private final XboxController manipulatorController = LOG.catchAll(() -> new XboxController(OIConstants.MANIPULATOR_CONTROLLER_PORT));
    
    // Xbox buttons
    // private final XboxTrigger ManipulatorXbox_TriggerL = LOG.catchAll(() -> new XboxTrigger(manipulatorController, XboxController.Axis.kLeftTrigger));
    // private final XboxTrigger ManipulatorXbox_TriggerR = LOG.catchAll(() -> new XboxTrigger(manipulatorController, XboxController.Axis.kRightTrigger));
    // private final JoystickButton ManipulatorXbox_A = LOG.catchAll(() -> new JoystickButton(manipulatorController, 1));
    // private final JoystickButton ManipulatorXbox_B = LOG.catchAll(() -> new JoystickButton(manipulatorController, 2));
    // private final JoystickButton ManipulatorXbox_X = LOG.catchAll(() -> new JoystickButton(manipulatorController, 3));
    // private final JoystickButton ManipulatorXbox_Y = LOG.catchAll(() -> new JoystickButton(manipulatorController, 4));
    // private final JoystickButton ManipulatorXbox_LB = LOG.catchAll(() -> new JoystickButton(manipulatorController, 5));
    // private final JoystickButton ManipulatorXbox_RB = LOG.catchAll(() -> new JoystickButton(manipulatorController, 6));
    // private final JoystickButton ManipulatorXbox_Back = LOG.catchAll(() -> new JoystickButton(manipulatorController, 7));
    // private final JoystickButton ManipulatorXbox_Start = LOG.catchAll(() -> new JoystickButton(manipulatorController, 8));
    // private final JoystickButton ManipulatorXbox_LStick = LOG.catchAll(() -> new JoystickButton(manipulatorController, 9));
    // private final JoystickButton ManipulatorXbox_RStick = LOG.catchAll(() -> new JoystickButton(manipulatorController, 10));
    
    // private final JoystickButton movementJoystick_Trigger = LOG.catchAll(() -> new JoystickButton(movementJoystick, 1));
    // private final JoystickButton movementJoystick_Button2 = LOG.catchAll(() -> new JoystickButton(movementJoystick, 2));
    // private final JoystickButton movementJoystick_Button3 = LOG.catchAll(() -> new JoystickButton(movementJoystick, 3));
    // private final JoystickButton movementJoystick_Button4 = LOG.catchAll(() -> new JoystickButton(movementJoystick, 4));
    // private final JoystickButton movementJoystick_Button5 = LOG.catchAll(() -> new JoystickButton(movementJoystick, 5));
    // private final JoystickButton movementJoystick_Button6 = LOG.catchAll(() -> new JoystickButton(movementJoystick, 6));
    // private final JoystickButton movementJoystick_Button7 = LOG.catchAll(() -> new JoystickButton(movementJoystick, 7));
    // private final JoystickButton movementJoystick_Button8 = LOG.catchAll(() -> new JoystickButton(movementJoystick, 8));
    private final JoystickButton movementJoystick_Button9 = LOG.catchAll(() -> new JoystickButton(movementJoystick, 9));
    // private final JoystickButton movementJoystick_Button10 = LOG.catchAll(() -> new JoystickButton(movementJoystick, 10));
    // private final JoystickButton movementJoystick_Button11 = LOG.catchAll(() -> new JoystickButton(movementJoystick, 11));
    
    // private final JoystickButton rotationJoystick_Trigger = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 1));
    // private final JoystickButton rotationJoystick_Button2 = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 2));
    // private final JoystickButton rotationJoystick_Button3 = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 3));
    // private final JoystickButton rotationJoystick_Button4 = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 4));
    // private final JoystickButton rotationJoystick_Button5 = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 5));
    // private final JoystickButton rotationJoystick_Button6 = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 6));
    // private final JoystickButton rotationJoystick_Button7 = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 7));
    // private final JoystickButton rotationJoystick_Button8 = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 8));
    private final JoystickButton rotationJoystick_Button9 = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 9));
    // private final JoystickButton rotationJoystick_Button10 = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 10));
    // private final JoystickButton rotationJoystick_Button11 = LOG.catchAll(() -> new JoystickButton(rotationJoystick, 11));

    public RobotContainer() {
        m_driveSubsystem.setDefaultCommand(new SwerveDrive(
            m_driveSubsystem, 
            () -> -modifyAxis(movementJoystick.getY()) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(movementJoystick.getX()) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis((rotationJoystick.getX() / 1.25)) * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            true));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        rotationJoystick_Button9.whenPressed(() -> m_driveSubsystem.zeroGyroscope());
        movementJoystick_Button9.whenPressed(() -> m_driveSubsystem.zeroGyroscope());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }

    private static double modifyAxis(double value) {
        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
}
