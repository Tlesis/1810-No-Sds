package frc.robot;

import static frc.robot.util.IO.*;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.    Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).    Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /**
     * The logger.
     * 
     * @since 2018
     */
    // private static final Logger LOG = new Logger();

    // The robot's subsystems
    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_driveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new SwerveDrive(
            m_driveSubsystem,
                () -> movementJoystick.getY(),
                () -> movementJoystick.getX(),
                () -> rotationJoystick.getX(),
                () -> true));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        rotationJoystick_Button9.onTrue(new InstantCommand(() -> m_driveSubsystem.gyro.zeroGyro()));
        movementJoystick_Button9.onTrue(new InstantCommand(() -> m_driveSubsystem.gyro.zeroGyro()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
