// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveDrive;
import frc.robot.log.Logger;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
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
    private static final Logger LOG = new Logger();
    
    // The robot's subsystems
    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

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

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_driveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new SwerveDrive(
                movementJoystick.getY(),
                movementJoystick.getX(),
                rotationJoystick.getX(),
                true,
                m_driveSubsystem));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
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
}
