package org.firstinspires.ftc.teamcode;

public interface Task {
    int TICK_MS = 50;
    void iterate();
    boolean hasNext();
}
