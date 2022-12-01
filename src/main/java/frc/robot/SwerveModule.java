package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX steerMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.DriveConstants.DRIVE_KS, Constants.DriveConstants.DRIVE_KV, Constants.DriveConstants.DRIVE_KA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configCanCoder();

        /* Angle Motor Config */
        steerMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.DriveConstants.MAX_SPEED;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.DriveConstants.WHEEL_CIRCUMFRENCE, Constants.DriveConstants.DRIVE_RATIO);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.DriveConstants.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.

        steerMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.DriveConstants.STEER_RATIO));
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(), Constants.DriveConstants.STEER_RATIO));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    private void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.DriveConstants.STEER_RATIO);
        steerMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configCanCoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        steerMotor.configFactoryDefault();
        steerMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        steerMotor.setInverted(Constants.DriveConstants.STEER_INVERT);
        steerMotor.setNeutralMode(Constants.DriveConstants.STEER_NEUTRAL_MODE);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.DriveConstants.DRIVE_INVERT);
        mDriveMotor.setNeutralMode(Constants.DriveConstants.DRIVE_NEUTRAL_MODE);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.DriveConstants.WHEEL_CIRCUMFRENCE, Constants.DriveConstants.DRIVE_RATIO), 
            getAngle()
        ); 
    }
}