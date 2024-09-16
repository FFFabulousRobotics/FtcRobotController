package org.firstinspires.ftc.teamcode;

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
