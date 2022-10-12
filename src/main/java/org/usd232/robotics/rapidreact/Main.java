package org.usd232.robotics.rapidreact;

import edu.wpi.first.wpilibj.RobotBase;

// https://drive.google.com/file/d/1SLhzzPlQXKFiCXgxzHFandjFaWhXx6js/view?usp=sharing

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
    private Main() {}

    /**
     * Main initialization function. Do not perform any initialization here.
     *
     * <p>If you change your main robot class, change the parameter type.
     */
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
