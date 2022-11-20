// Based on Code from Team364 - BaseFalconSwerve
// https://github.com/Team364/BaseFalconSwerve/tree/338c0278cb63714a617f1601a6b9648c64ee78d1

package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.Utils;

import static frc.robot.Constants.*;

public class SwerveModule {

    public final WPI_TalonFX steerMotor;
    public final WPI_TalonFX driveMotor;
    public final WPI_CANCoder canCoder;

    public final int moduleNumber;
    private final double offset;
    private Rotation2d lastAngle;

    SimpleMotorFeedforward feedforward =
        new SimpleMotorFeedforward(ModuleConstants.DRIVE_kS, ModuleConstants.DRIVE_kV, ModuleConstants.DRIVE_kA);

    public SwerveModule(int moduleNumber,
                 int driveMotorID,
                 int steerMotorID,
                 int canCoderID,
                 double steerOffset) {

        this.moduleNumber = moduleNumber;
        offset = steerOffset;

        canCoder = new WPI_CANCoder(canCoderID);
        configCANCoderMotor();

        driveMotor = new WPI_TalonFX(driveMotorID);
        configDriveMotor();

        steerMotor = new WPI_TalonFX(steerMotorID);
        configSteerMotor();

        lastAngle = getState().angle;
    }

    // FIXME: Might Snap back
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = Utils.optimize(desiredState, getState().angle);

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / ModuleConstants.MAX_VELOCITY;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity =
                Utils.MPSToFalcon(
                    desiredState.speedMetersPerSecond,
                    (ModuleConstants.WHEEL_DIAMETER * Math.PI),
                    ModuleConstants.DRIVE_REDUCTION);
            driveMotor.set(
                ControlMode.Velocity,
                velocity,
                DemandType.ArbitraryFeedForward,
                feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        // Prevent rotating module if speed is less then 1% and the angle change is less than 5deg.
        // Prevents Jittering.
        Rotation2d angle = desiredState.angle;
        if ((Math.abs(desiredState.speedMetersPerSecond) < (ModuleConstants.MAX_VELOCITY * 0.01))
                && Math.abs(lastAngle.minus(angle).getDegrees()) < 5) {
            angle = lastAngle;
        }

        steerMotor.set(
            ControlMode.Position,
            Utils.degreesToFalcon(angle.getDegrees(), ModuleConstants.STEER_REDUCTION));
        lastAngle = angle;
    }

    public Rotation2d getCanCoderAngle() {
        return Rotation2d.fromDegrees(canCoder.getAbsolutePosition());
    }

    public Rotation2d getTargetAngle() {
        return lastAngle;
    }

    public SwerveModuleState getState() {
        double velocity =
            Utils.falconToMPS(
                driveMotor.getSelectedSensorVelocity(),
                (ModuleConstants.WHEEL_DIAMETER * Math.PI),
                ModuleConstants.DRIVE_REDUCTION);

        Rotation2d angle =
            Rotation2d.fromDegrees(
                Utils.falconToDegrees(
                    steerMotor.getSelectedSensorPosition(),
                    ModuleConstants.STEER_REDUCTION));
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double position =
                Utils.FalconToMeters(
                        driveMotor.getSelectedSensorPosition(),
                        (ModuleConstants.WHEEL_DIAMETER * Math.PI),
                        ModuleConstants.DRIVE_REDUCTION);
        Rotation2d angle =
                Rotation2d.fromDegrees(
                        Utils.falconToDegrees(
                                steerMotor.getSelectedSensorPosition(),
                                ModuleConstants.STEER_REDUCTION));
        return new SwerveModulePosition(position, angle);
    }

    //******************** UTIL ********************/
    public void resetToAbsolute() {
        double absolutePosition =
            Utils.degreesToFalcon(getCanCoderAngle().getDegrees() - this.offset, ModuleConstants.STEER_REDUCTION);
        steerMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configCANCoderMotor() {
        canCoder.configFactoryDefault();

        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.sensorDirection = ModuleConstants.CANCODER_INVERTED;
        config.initializationStrategy =
            SensorInitializationStrategy.BootToAbsolutePosition;
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        canCoder.configAllSettings(config);

        canCoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 249);
        canCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 253);
    }

    private void configSteerMotor() {
        steerMotor.configFactoryDefault();

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.slot0.kP = ModuleConstants.STEER_kP;
        config.slot0.kI = ModuleConstants.STEER_kI;
        config.slot0.kD = ModuleConstants.STEER_kD;
        config.slot0.allowableClosedloopError = ModuleConstants.ALLOWED_ERROR;
        config.supplyCurrLimit.currentLimit = ModuleConstants.STEER_CURRENT_LIMIT;
        config.supplyCurrLimit.enable = true;
        config.initializationStrategy = SensorInitializationStrategy.BootToZero;
        steerMotor.configAllSettings(config);

        steerMotor.setInverted(ModuleConstants.STEER_INVERTED);
        steerMotor.setNeutralMode(ModuleConstants.STEER_NEUTRAL_MODE);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        driveMotor.configFactoryDefault();

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.slot0.kP = ModuleConstants.DRIVE_kP;
        config.slot0.kI = ModuleConstants.DRIVE_kI;
        config.slot0.kD = ModuleConstants.DRIVE_kD;
        config.supplyCurrLimit.currentLimit = ModuleConstants.DRIVE_CURRENT_LIMIT;
        config.supplyCurrLimit.enable = true;
        config.initializationStrategy = SensorInitializationStrategy.BootToZero;
        config.openloopRamp = ModuleConstants.OPEN_LOOP_RAMP;
        config.closedloopRamp = ModuleConstants.CLOSED_LOOP_RAMP;
        driveMotor.configAllSettings(config);

        driveMotor.setInverted(ModuleConstants.DRIVE_INVERTED);
        driveMotor.setNeutralMode(ModuleConstants.DRIVE_NEUTRAL_MODE);
        driveMotor.setSelectedSensorPosition(0);
    }
}
