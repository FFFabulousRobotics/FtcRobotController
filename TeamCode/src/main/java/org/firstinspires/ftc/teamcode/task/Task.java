package org.firstinspires.ftc.teamcode.task;

public interface Task {
    int TICK_MS = 50;

    default void init() {
    }

    void iterate();

    default void finish() {
    }

    boolean hasNext();

    default void cancel() {
    }
}
