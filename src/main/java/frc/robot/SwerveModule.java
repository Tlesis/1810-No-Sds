package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import static frc.robot.Constants.*;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

public class SwerveModule {
    
    private WPI_TalonFX axisMotor;
    private WPI_TalonFX driveMotor;
    private double canCoderAngle;
    private CANCoder canCoder;

    // Note: Phoenix Lib init knocks motors out of alignment
    // Wait until you see that on the console before running, else realign

    public SwerveModule(int turningMotor, int driveMotor, int canCoder) {
        this.canCoder = new CANCoder(canCoder);
        this.canCoderAngle = 0.0;

        this.axisMotor = new WPI_TalonFX(turningMotor);
        this.axisMotor.setInverted(TalonFXInvertType.Clockwise); // FIXME
        this.axisMotor.configNeutralDeadband(0.0001);
        this.axisMotor.configSelectedFeedbackSensor(FeedbackDevice.valueOf(1));

        this.axisMotor.setSelectedSensorPosition(-(this.canCoderAngle/360) * 2048 * ModuleConstants.STEER_RATIO);
        this.axisMotor.config_kF(0, ModuleConstants.STEER_kF);
        this.axisMotor.config_kP(0, ModuleConstants.STEER_kP);
        this.axisMotor.config_kI(0, ModuleConstants.STEER_kI);
        this.axisMotor.config_kD(0, ModuleConstants.STEER_kD);
        this.axisMotor.setNeutralMode(NeutralMode.Coast);

        this.driveMotor = new WPI_TalonFX(driveMotor);
        this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        this.driveMotor.setInverted(TalonFXInvertType.CounterClockwise); // FIXME
        this.driveMotor.configNeutralDeadband(0.01);
        this.driveMotor.setSelectedSensorPosition(0);
        this.driveMotor.config_kF(0, ModuleConstants.DRIVE_kF);
        this.driveMotor.config_kP(0, ModuleConstants.DRIVE_kP);
        this.driveMotor.config_kI(0, ModuleConstants.DRIVE_kI);
        this.driveMotor.config_kD(0, ModuleConstants.DRIVE_kD);
        this.driveMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void set(double speed, double angle) {
        double steerEncoderAngle = axisMotor.getSelectedSensorPosition() / ModuleConstants.STEER_RATIO / 2048 * (2 * PI);
        double driveConstants = 204.8 / (2 * PI) * ModuleConstants.DRIVE_RATIO / ModuleConstants.WHEEL_RADIUS_METERS;
        double angleConstants = 2048 / (2 * PI) * ModuleConstants.STEER_RATIO;

        speed *= driveConstants;

        double encoderTrue = steerEncoderAngle % (2 * PI);
        double dTheta = angle - encoderTrue;

        if (abs(-2 * PI + dTheta) < abs(dTheta)) {
            if (abs(-2 * PI + dTheta) < abs(2 * PI + dTheta)) {
                dTheta = -2 * PI + dTheta;
            } else {
                dTheta = 2 * PI + dTheta;
            }
        } else if (abs(dTheta) > abs(2*PI + dTheta)) {
            dTheta = 2 * PI + dTheta;
        }

        double angleFinal = steerEncoderAngle + dTheta;
        angleFinal *= angleConstants;

        this.driveMotor.set(ControlMode.Velocity, speed);
        if (speed < 120) {
            this.axisMotor.set(ControlMode.Velocity, 0);
            this.driveMotor.set(ControlMode.Velocity, 0);
        } else {
            this.axisMotor.set(ControlMode.Position, angleFinal);
            this.driveMotor.set(ControlMode.Velocity, speed);
        }
    }

    public void zero() {
        canCoderAngle = this.canCoder.getAbsolutePosition();
        this.axisMotor.setSelectedSensorPosition(-(canCoderAngle / 360) * 2048 * ModuleConstants.STEER_RATIO);
        this.axisMotor.set(ControlMode.Velocity, 0);
    }
}
