package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs() {
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.DriveConstants.angleEnableCurrentLimit, 
            Constants.DriveConstants.angleContinuousCurrentLimit, 
            Constants.DriveConstants.anglePeakCurrentLimit, 
            Constants.DriveConstants.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = Constants.DriveConstants.STEER_KP;
        swerveAngleFXConfig.slot0.kI = Constants.DriveConstants.STEER_KI;
        swerveAngleFXConfig.slot0.kD = Constants.DriveConstants.STEER_KD;
        swerveAngleFXConfig.slot0.kF = Constants.DriveConstants.STEER_KF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.DriveConstants.driveEnableCurrentLimit, 
            Constants.DriveConstants.driveContinuousCurrentLimit, 
            Constants.DriveConstants.drivePeakCurrentLimit, 
            Constants.DriveConstants.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.DriveConstants.DRIVE_KP;
        swerveDriveFXConfig.slot0.kI = Constants.DriveConstants.DRIVE_KI;
        swerveDriveFXConfig.slot0.kD = Constants.DriveConstants.DRIVE_KD;
        swerveDriveFXConfig.slot0.kF = Constants.DriveConstants.DRIVE_KF;
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.DriveConstants.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.DriveConstants.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.DriveConstants.CANCODER_INVERT;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}