package org.usd232.robotics.rapidreact.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.usd232.robotics.rapidreact.Constants.ShooterConstants;

// https://drive.google.com/file/d/1Big3GqA8ZGPYKn6hLIk1PlT94j46iKOF/view?usp=sharing

public class ShooterSubsystem extends SubsystemBase {

    private static final CANSparkMax shooter = new CANSparkMax(ShooterConstants.MOTOR_PORT, MotorType.kBrushless);
    private static final RelativeEncoder shooterEncoder = shooter.getEncoder();
    private static final SparkMaxPIDController shooterPID = shooter.getPIDController();

    public ShooterSubsystem() {
        shooterPID.setP(ShooterConstants.kP);
        shooterPID.setFF(ShooterConstants.kFF);
        shooter.setInverted(true);
        shooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
        shooter.burnFlash();
        shooterPID.setOutputRange(0, 1);
    }

    /** 
     * Turns on the shooter
     * @param percent
     */
    public void shooterOn(double percent) {
        if (percent > 1) {
            percent = 1;
        } else if (percent < -1) {
            percent = -1;
        }
        shooter.set(percent); // 0.435 good for auto
    }

    /**
     * Sets the shooter motor to a specified percent speed
     * @param percent the percent you want to run the motor at <p>{@code [-1, 1]}
     */
    public void setShooterVelocity(double percent) { // TODO: Test
        /* if (percent > 1) {
            percent = 1;
        } else if (percent < -1) {
            percent = -1;
        }
        percent *= ShooterConstants.MAX_SPEED; */

        velocity(percent);
    }

    /** Turns off the shooter. Woah. */
    public void shooterOff() {
        shooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
        shooter.set(0);
        shooterEncoder.setPosition(0);
    }

    /**
     * Sets the shooter to a desired RPM
     * @param velocity The RPM that you want to run at <p>{@code [-MAX_SPEED, MAX_SPEED]}
     */
    private void velocity(double velocity) {
        shooterPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }

    /** @return The position of the encoders. */
    public static double getEncoderPosition() {
        return shooterEncoder.getPosition();
    } 

    /** @return The velocity of the encoders. */
    public static double getEncoderVelocity() {
        return shooterEncoder.getVelocity();
    }
}
