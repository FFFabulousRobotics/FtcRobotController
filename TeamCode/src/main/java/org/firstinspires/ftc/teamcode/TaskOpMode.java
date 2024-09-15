package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;

public abstract class TaskOpMode extends LinearOpMode {

    List<Task> tasks;
    Gamepad gamepad1Snapshot;
    Gamepad gamepad2Snapshot;

    public TaskOpMode() {
        tasks = new ArrayList<>();
    }

    public abstract void linearInit();

    public void linearStart() {};

    public abstract void linearLoop();

    public void linearStop() {};

    @Override
    public void runOpMode() {
        linearInit();
        waitForStart();
        linearStart();
        while (opModeIsActive()) {
            for (Task task : tasks) {
                if (!task.hasNext()) {
                    task.finish();
                    tasks.remove(task);
                }
                task.iterate();
            }
            linearLoop();
            gamepad1Snapshot.copy(gamepad1);
            gamepad2Snapshot.copy(gamepad2);
            sleep(Task.TICK_MS);
        }
        linearStop();
    }

    public void registerTask(Task task) {
        task.init();
        tasks.add(task);
    }
}
