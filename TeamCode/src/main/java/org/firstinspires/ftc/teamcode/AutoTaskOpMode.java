package org.firstinspires.ftc.teamcode;

import android.util.ArrayMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Map;

public abstract class AutoTaskOpMode extends LinearOpMode {

    Map<Integer, Task> taskMap;
    Map<Integer, Integer> taskLinks;
    // format:<int: thisTaskId, int: lastTaskId>
    Map<Integer, Task> currentTasks;
    int minimumFreeTaskId = 0;

    public AutoTaskOpMode() {
        taskMap = new ArrayMap<>();
        taskLinks = new ArrayMap<>();
    }

    public abstract void linearInit();

    public abstract void linearStart();

    public void linearLoop(){}

    public void linearStop() {
    }

    @Override
    public void runOpMode() {
        linearInit();
        waitForStart();
        linearStart();
        while (opModeIsActive()) {
            for (Map.Entry<Integer, Task> taskEntry: taskMap.entrySet()) {
                int taskId = taskEntry.getKey();
                Task task = taskEntry.getValue();
                if (!task.hasNext()) {
                    task.finish();
                    taskMap.remove(taskId, task);

                    //I don't know whether it can run correctly...
                    //TODO: still need testing
                    for (Map.Entry<Integer, Integer> eachLink: taskLinks.entrySet()) {
                        int eachTaskId = eachLink.getKey();
                        int lastTaskId = eachLink.getValue();
                        if(lastTaskId == taskId){
                            runTask(eachTaskId);
                        }
                    }
                    continue;
                }
//                TaskStatus taskStatus = taskLinks.get(taskId);
//                assert taskStatus != null;
//                if (taskMap.containsKey(taskStatus.afterTaskId)) continue;
//                boolean shouldRunTask = taskStatus.tickDown();
//                if (!shouldRunTask) continue;
                task.iterate();
            }
            linearLoop();
            sleep(Task.TICK_MS);
        }
        linearStop();
    }

    private boolean isRunning(){
        return taskMap.isEmpty();
    }

    public int createTask(Task task,int linkedTaskId){
        int taskId = minimumFreeTaskId;
        taskMap.put(taskId, task);
        taskLinks.put(taskId,linkedTaskId);
        minimumFreeTaskId++;
        return taskId;
    }
    public int createTask(Task task){
        return createTask(task,minimumFreeTaskId - 1);
    }

    public void runTask(int taskId){
        Task task = taskMap.get(taskId);
        task.init();
        currentTasks.put(taskId, task);
    }

//    public int scheduleTask(Task task) {
//        return scheduleTask(task, 0);
//    }
//
//    public int scheduleTask(Task task, int delayTicks) {
//        return scheduleTask(task, delayTicks, 1);
//    }
//
//    public int scheduleTask(Task task, int delayTicks, int loopTicks) {
//        return scheduleTask(task, delayTicks, loopTicks, -1);
//    }
//
//    public int scheduleTask(Task task, int delayTicks, int loopTicks, int afterTaskId) {
//        task.init();
//        int taskId = minimumFreeTaskId;
//        TaskStatus taskStatus = new TaskStatus(delayTicks, loopTicks, afterTaskId);
//        taskMap.put(taskId, task);
//        taskLinks.put(taskId, taskStatus);
//        minimumFreeTaskId++;
//        return taskId;
//    }
//
//    public void cancelTask(int taskId, boolean chained) {
//        Task task = taskMap.get(taskId);
//        if (task == null) return;
//        task.cancel();
//        taskMap.remove(taskId, task);
//        taskLinks.remove(taskId);
//        if (chained) {
//            taskLinks.entrySet().stream()
//                    .filter(e -> e.getValue().afterTaskId == taskId)
//                    .forEach(e -> cancelTask(e.getKey(), true));
//        }
//    }
}
