package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;

    private final CANCoder canCoder;

    private final PIDController steerPIDController;

    private final double moduleOffset;

    private final boolean canCoderReversed;

    private final String moduleName;

    public SwerveModule(int driveMotorID, int steerMotorID, int canCoderID, 
                        double moduleOffset, boolean driveMotorReversed,
                        boolean steerMotorReversed, boolean canCoderReversed, String moduleName) {

        this.driveMotor = new TalonFX(driveMotorID);
        this.steerMotor = new TalonFX(steerMotorID);

        driveMotor.setInverted(driveMotorReversed);
        steerMotor.setInverted(steerMotorReversed);

        this.canCoder = new CANCoder(canCoderID);

        this.moduleOffset = moduleOffset;

        this.canCoderReversed = canCoderReversed;

        this.steerPIDController = new PIDController(ModuleConstants.P, 0.0, ModuleConstants.D);
        steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

        this.moduleName = moduleName;

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition();
    }

    public double getSteerPosition() {
        return steerMotor.getSelectedSensorPosition();
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity();
    }

    public double getSteerVelocity() {
        return steerMotor.getSelectedSensorVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = canCoder.getPosition();
        angle *= 2.0 * Math.PI;
        angle -= moduleOffset;
        return angle * (canCoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        steerMotor.setSelectedSensorPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
        steerMotor.set(ControlMode.PercentOutput, steerPIDController.calculate(getSteerPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + this.moduleName + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        steerMotor.set(ControlMode.PercentOutput, 0);
    }
}