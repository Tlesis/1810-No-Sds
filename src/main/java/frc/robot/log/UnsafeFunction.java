package frc.robot.log;

@FunctionalInterface
public interface UnsafeFunction<T> {
    T run() throws Throwable;
}
