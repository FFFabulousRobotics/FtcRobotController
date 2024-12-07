package org.firstinspires.ftc.teamcode.advancedManual;


import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

public class TaskManager {
    protected int currentMaxId = 0;
    protected HashMap<Integer, Task> tasks;
    protected HashMap<Integer, Boolean> taskState;

    public TaskManager() {
        tasks = new HashMap<>();
        taskState = new HashMap<>();
    }

    public void updateServos(){
        Set<Integer> tasksToRemove = new HashSet<>();
        for (HashMap.Entry<Integer, Task> taskEntry : tasks.entrySet()) {
            int taskId = taskEntry.getKey();
            Task task = taskEntry.getValue();
            if (!task.hasNext()) {
                task.finish();
                taskState.replace(taskId, false);
                tasksToRemove.add(taskId);
                continue;
            }
            task.iterate();
        }
        for (int taskId : tasksToRemove) {
            tasks.remove(taskId);
        }
    }

    public int setTimedServoPosition(Servo servo, double position, long timeMs) {
        final double deltaPos = position - servo.getPosition();
        final long iterationCount = timeMs / Task.TICK_MS;
        final double positionPerIteration = deltaPos / iterationCount;
        Task task =  new Task() {
            long remainingIterations = iterationCount;
            final double targetPosition = position;

            @Override
            public void iterate() {
                servo.setPosition(servo.getPosition() + positionPerIteration);
                remainingIterations--;
            }

            @Override
            public void finish() {
                servo.setPosition(targetPosition);
            }

            @Override
            public boolean hasNext() {
                return remainingIterations > 0;
            }
        };
        return registerTask(task);
    }
    public int await(long timeMs){
        Task task =  new Task() {
            long iteration = timeMs / 50;
            @Override
            public void iterate() {
                iteration--;
            }

            @Override
            public boolean hasNext() {
                return iteration > 0;
            }
        };
        return registerTask(task);
    }
    public boolean getTaskState(int taskId){
        return Boolean.TRUE.equals(taskState.get(taskId));
    }
    private int registerTask(Task task){
        int taskId = findMinFreeTaskId();
        tasks.put(taskId, task);
        taskState.put(taskId, true);
        return taskId;
    }
    private int findMinFreeTaskId() {
        currentMaxId += 1;
        return currentMaxId;
    }
}