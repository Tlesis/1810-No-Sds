// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANCoder canCoder;

    private final boolean canCoderReversed;

    private final PIDController drivePIDController =
        new PIDController(ModuleConstants.kP_STEER, 0, ModuleConstants.kD_STEER);

    // Using a TrapezoidProfile PIDController to allow for smooth turning
    private final ProfiledPIDController turningPIDController =
        new ProfiledPIDController(
            ModuleConstants.kP_DRIVE,
            0,
            0,
            new TrapezoidProfile.Constraints(
                ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
                ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel The channel of the drive motor.
     * @param steerMotorChannel The channel of the turning motor.
     * @param driveMotorReversed Whether the drive motor is reversed.
     * @param steerMotorReversed Whether the steer motor is reversed.
     * @param canCoderReversed Whether the CANCoder is reversed.
     */
    public SwerveModule(
        int driveMotorChannel,
        int steerMotorChannel,
        int canCoderChannel,
        boolean driveMotorReversed,
        boolean steerMotorReversed,
        boolean canCoderReversed) {
        driveMotor = new TalonFX(driveMotorChannel);
        steerMotor = new TalonFX(steerMotorChannel);

        /** Set the distance per pulse for the drive encoder. We can simply use the
        distance traveled for one rotation of the wheel divided by the encoder resolution. */
        driveMotor.configSelectedFeedbackCoefficient(ModuleConstants.DRIVE_DISTANCE_PER_PULSE);

        driveMotor.setInverted(driveMotorReversed);
        steerMotor.setInverted(steerMotorReversed);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        canCoder = new CANCoder(canCoderChannel);

        this.canCoderReversed = canCoderReversed;
    }

    private double getCanCoderPosition() {
        double value = canCoder.getPosition();
        return value *= (this.canCoderReversed) ? -1.0 : 1.0;
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getSelectedSensorVelocity(), new Rotation2d(getCanCoderPosition()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
            SwerveModuleState.optimize(desiredState, new Rotation2d(getCanCoderPosition()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput =
            drivePIDController.calculate(driveMotor.getSelectedSensorVelocity(), state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput =
            turningPIDController.calculate(getCanCoderPosition(), state.angle.getRadians());

        // Calculate the turning motor output from the turning PID controller.
        driveMotor.set(TalonFXControlMode.PercentOutput, driveOutput);
        steerMotor.set(TalonFXControlMode.Position, turnOutput);
    }

    public void stop() {
        driveMotor.set(TalonFXControlMode.PercentOutput, 0);
        steerMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
}
