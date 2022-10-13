package org.usd232.robotics.rapidreact;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.ErrorCode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import static org.usd232.robotics.rapidreact.Constants.*;

public class SwerveModule {

    private final TalonFX steerMotor;
    private final TalonFX driveMotor;
    private final CANCoder absoluteEncoder;

    private double resetIteration = 0;
    private double referenceAngleRadians = 0;

    public SwerveModule(ShuffleboardLayout layout,
                        int steerPort,
                        int drivePort,
                        int absoluteEncoderPort,
                        double steerOffset) {
        // Setup drive motor
        TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration();
        driveMotor = new TalonFX(drivePort);
        checkCtreError(driveMotor.configAllSettings(driveMotorConfiguration), "Failed to configure Falcon 500");
        driveMotor.setNeutralMode(NeutralMode.Coast); // TODO
        driveMotor.setInverted(TalonFXInvertType.Clockwise);
        driveMotor.setSensorPhase(true);

        // setup steerMotor
        TalonFXConfiguration steerMotorConfiguration = new TalonFXConfiguration();
        steerMotorConfiguration.slot0.kP = 0.2;
        steerMotorConfiguration.slot0.kI = 0.0;
        steerMotorConfiguration.slot0.kD = 0.1;
        steerMotor = new TalonFX(steerPort);
        checkCtreError(steerMotor.configAllSettings(steerMotorConfiguration, 250), "Failed to configure Falcon 500 settings");
        checkCtreError(steerMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 250), "Failed to set Falcon 500 feedback sensor");
        steerMotor.setSensorPhase(true);
        steerMotor.setInverted(TalonFXInvertType.CounterClockwise);
        steerMotor.setNeutralMode(NeutralMode.Brake);
        checkCtreError(steerMotor.setSelectedSensorPosition(getAbsoluteAngle() / ModuleConstants.MOTOR_ENCODER_POSITION_COEFFICIENT, 0, 250), "Failed to set Falcon 500 encoder position");

        // Set up CANCoder
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = Math.toDegrees(steerOffset);
        config.sensorDirection = false;

        // Check for CANCoder errors
        absoluteEncoder = new CANCoder(absoluteEncoderPort);
        checkCtreError(absoluteEncoder.configAllSettings(config, 250), "Failed to configure CANCoder");
        checkCtreError(absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 250), "Failed to configure CANCoder update rate");

        // TODO: IDK if this will do what we want
        layout.addNumber("Absolute Encoder Angle", () -> Math.toDegrees(getAbsoluteAngle()));
        layout.addNumber("Current Velocity", () -> getDriveVelocity());
        layout.addNumber("Current Angle", () -> Math.toDegrees(getSteerAngle()));
        layout.addNumber("Target Angle", () -> Math.toDegrees(referenceAngleRadians));
    }

    public void set(double driveVoltage, double steerAngle) {
        /* if (Math.abs(driveVoltage / 12) < 0.001) {
            driveMotor.setNeutralMode(NeutralMode.Coast);
        } else {
            driveMotor.setNeutralMode(NeutralMode.Brake);
        } */

        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {
            steerAngle += 2.0 * Math.PI;
        }

        double difference = steerAngle - getSteerAngle();
        // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
        if (difference >= Math.PI) {
            steerAngle -= 2.0 * Math.PI;
        } else if (difference < -Math.PI) {
            steerAngle += 2.0 * Math.PI;
        }
        difference = steerAngle - getSteerAngle(); // Recalculate difference

        // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
        // movement of the module is less than 90 deg
        if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
            // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
            steerAngle += Math.PI;
            driveVoltage *= -1.0;
        }

        // Put the target angle back into the range [0, 2pi)
        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {
            steerAngle += 2.0 * Math.PI;
        }

        setReferenceVoltage(driveVoltage);
        setReferenceAngle(steerAngle, driveVoltage);
    }

    private double getSteerAngle() {
        double motorAngleRadians = steerMotor.getSelectedSensorPosition() * ModuleConstants.MOTOR_ENCODER_POSITION_COEFFICIENT;
        motorAngleRadians %= 2.0 * Math.PI;
        if (motorAngleRadians < 0.0) {
            motorAngleRadians += 2.0 * Math.PI;
        }

        return motorAngleRadians;
    }

    private void setReferenceVoltage(double voltage) {
        driveMotor.set(TalonFXControlMode.PercentOutput, voltage / 12);
    }

    private void setReferenceAngle(double referenceAngleRadians, double speed) {
        if (speed / 12 == 0) return;

        double currentAngleRadians = steerMotor.getSelectedSensorPosition() * ModuleConstants.MOTOR_ENCODER_POSITION_COEFFICIENT;

        // Reset the NEO's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter anymore.
        if (steerMotor.getSelectedSensorVelocity() * ModuleConstants.MOTOR_ENCODER_VELOCITY_COEFFICENT < ModuleConstants.ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++resetIteration >= ModuleConstants.ENCODER_RESET_ITERATIONS) {
                resetIteration = 0;
                double absoluteAngle = getAbsoluteAngle();
                steerMotor.setSelectedSensorPosition(absoluteAngle / ModuleConstants.MOTOR_ENCODER_POSITION_COEFFICIENT);
                currentAngleRadians = absoluteAngle;
            }
        } else {
            resetIteration = 0;
        }

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Falcon's encoder can go above that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;            if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        steerMotor.set(TalonFXControlMode.Position, adjustedReferenceAngleRadians / ModuleConstants.MOTOR_ENCODER_POSITION_COEFFICIENT);

        this.referenceAngleRadians = referenceAngleRadians;
    }

    private static void checkCtreError(ErrorCode errorCode, String message) {
        if (RobotBase.isReal() && errorCode != ErrorCode.OK) {
            DriverStation.reportError(String.format("%s: %s", message, errorCode.toString()), false);
        }
    }

    /** util */
    public double[] getSelectedSensorPosition() {
        return new double[] {driveMotor.getSelectedSensorPosition(),
                             steerMotor.getSelectedSensorPosition()};
    }

    public double getAbsoluteAngle() {
        double angle = Math.toRadians(absoluteEncoder.getAbsolutePosition());
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity() * ModuleConstants.MOTOR_ENCODER_VELOCITY_COEFFICENT;
    }
}
