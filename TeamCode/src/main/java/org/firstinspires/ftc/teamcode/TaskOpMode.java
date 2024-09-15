package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

public abstract class TaskOpMode extends LinearOpMode {

    List<Task> tasks;

    public TaskOpMode() {
        tasks = new ArrayList<>();
    }

    @Override
    public void runOpMode() {
        for (Task task: tasks) {
            if (!task.hasNext()) {
                tasks.remove(task);
            }
            task.iterate();
        }
        sleep(Task.TICK_MS);
    }
}
