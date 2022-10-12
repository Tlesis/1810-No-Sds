package org.usd232.robotics.rapidreact.log;

@FunctionalInterface
public interface UnsafeFunction<T> {
    T run() throws Throwable;
}
