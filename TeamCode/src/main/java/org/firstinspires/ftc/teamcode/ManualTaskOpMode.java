package org.firstinspires.ftc.teamcode;

import android.util.ArrayMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Map;

public abstract class ManualTaskOpMode extends LinearOpMode {

    Map<Integer, Task> tasks;
    int minimumFreeTaskId = 0;
    final int TICK_MS = 50;

    public ManualTaskOpMode() {
        tasks = new ArrayMap<>();
    }

    public abstract void linearInit();

    public void linearStart(){}

    public abstract void linearLoop();

    public void linearStop() {
    }

    @Override
    public void runOpMode() {
        linearInit();
        waitForStart();
        linearStart();
        while (opModeIsActive()) {
            for (Map.Entry<Integer, Task> taskEntry: tasks.entrySet()) {
                int taskId = taskEntry.getKey();
                Task task = taskEntry.getValue();
                if (!task.hasNext()) {
                    task.finish();
                    tasks.remove(taskId, task);
                    continue;
                }
                task.iterate();
            }
            linearLoop();
            sleep(TICK_MS);
        }
        linearStop();
    }

    public boolean isRunning(){
        return tasks.isEmpty();
    }
    public int createTask(Task task){
        task.init();
        int taskId = minimumFreeTaskId;
        tasks.put(taskId, task);
        minimumFreeTaskId++;
        return taskId;
    }

//    public void cancelTask(int taskId, boolean chained) {
//        Task task = tasks.get(taskId);
//        if (task == null) return;
//        task.cancel();
//        tasks.remove(taskId, task);
//        taskStatuses.remove(taskId);
//        if (chained) {
//            taskStatuses.entrySet().stream()
//                    .filter(e -> e.getValue().afterTaskId == taskId)
//                    .forEach(e -> cancelTask(e.getKey(), true));
//        }
//    }
}

