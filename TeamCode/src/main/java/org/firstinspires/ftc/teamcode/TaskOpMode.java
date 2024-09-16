package org.firstinspires.ftc.teamcode;

import android.util.ArrayMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Map;

public abstract class TaskOpMode extends LinearOpMode {

    Map<Integer, Task> tasks;
    Map<Integer, TaskStatus> taskStatuses;
    int minimumFreeTaskId = 0;
    Gamepad gamepad1Snapshot;
    Gamepad gamepad2Snapshot;

    public TaskOpMode() {
        tasks = new ArrayMap<>();
        taskStatuses = new ArrayMap<>();
    }

    public abstract void linearInit();

    public void linearStart() {
    }

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
                    taskStatuses.remove(taskId);
                    continue;
                }
                TaskStatus taskStatus = taskStatuses.get(taskId);
                assert taskStatus != null;
                if (tasks.containsKey(taskStatus.afterTaskId)) continue;
                boolean shouldRunTask = taskStatus.tickDown();
                if (!shouldRunTask) continue;
                task.iterate();
            }
            linearLoop();
            sleep(Task.TICK_MS);
        }
        linearStop();
    }

    public int scheduleTask(Task task) {
        return scheduleTask(task, 0);
    }

    public int scheduleTask(Task task, int delayTicks) {
        return scheduleTask(task, delayTicks, 1);
    }

    public int scheduleTask(Task task, int delayTicks, int loopTicks) {
        return scheduleTask(task, delayTicks, loopTicks, -1);
    }

    public int scheduleTask(Task task, int delayTicks, int loopTicks, int afterTaskId) {
        task.init();
        int taskId = minimumFreeTaskId;
        TaskStatus taskStatus = new TaskStatus(delayTicks, loopTicks, afterTaskId);
        tasks.put(taskId, task);
        taskStatuses.put(taskId, taskStatus);
        minimumFreeTaskId++;
        return taskId;
    }

    public void cancelTask(int taskId, boolean chained) {
        Task task = tasks.get(taskId);
        if (task == null) return;
        task.cancel();
        tasks.remove(taskId, task);
        taskStatuses.remove(taskId);
        if (chained) {
            taskStatuses.entrySet().stream()
                    .filter(e -> e.getValue().afterTaskId == taskId)
                    .forEach(e -> cancelTask(e.getKey(), true));
        }
    }
}
