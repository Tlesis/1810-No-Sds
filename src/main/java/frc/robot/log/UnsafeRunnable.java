package frc.robot.log;

@FunctionalInterface
public interface UnsafeRunnable {
    void run() throws Throwable;
}
