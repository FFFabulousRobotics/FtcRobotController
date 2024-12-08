package org.firstinspires.ftc.teamcode.advancedManual.test;

import android.util.ArrayMap;

import org.firstinspires.ftc.teamcode.advancedManual.Task;

import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

class ServoStimulator{
    double position = 0;

    public void setPosition(double position) {
        this.position = position;
        System.out.println(this.position);
    }

    public double getPosition() {
        return position;
    }
}

class ServoMgrTest {
    protected Map<Integer, Task> tasks;
    protected Map<Integer, Boolean> taskState;

    public ServoMgrTest() {
        tasks = new ArrayMap<>();
    }

    public void updateServos(){
        for (Map.Entry<Integer, Task> taskEntry : tasks.entrySet()) {
            int taskId = taskEntry.getKey();
            Task task = taskEntry.getValue();
            if (!task.hasNext()) {
                task.finish();
                taskState.replace(taskId, false);
                tasks.remove(taskId, task);
                continue;
            }
            task.iterate();
        }
    }

    private Task timedSetPosition(ServoStimulator servo, double position, long timeMs) {
        final double deltaPos = position - servo.getPosition();
        final long iterationCount = timeMs / Task.TICK_MS;
        final double positionPerIteration = deltaPos / iterationCount;
        return new Task() {
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
    }
    public int setTimedServoPosition(ServoStimulator servo, double position, long timeMs){
        Task task = timedSetPosition(servo, position, timeMs);
        int taskId = findMinFreeTaskId();
        tasks.put(taskId, task);
        return taskId;
    }
    private int findMinFreeTaskId() {
        List<Integer> taskIdList = tasks.keySet().stream().sorted().collect(Collectors.toList());
        int maxTaskId = taskIdList.get(taskIdList.size() - 1);
        for (int i = 0; i <= maxTaskId; i++) {
            if (taskIdList.contains(i)) return 1;
        }
        return maxTaskId + 1;
    }
}

class Main{
    public static void main(String[] args) {
        ServoMgrTest servoMgr = new ServoMgrTest();
        ServoStimulator servo1 = new ServoStimulator();
        ServoStimulator servo2 = new ServoStimulator();
        ServoStimulator servo3 = new ServoStimulator();
        servoMgr.setTimedServoPosition(servo1, 0.7, 1000);
        servoMgr.setTimedServoPosition(servo2, 0.2, 5000);
        servoMgr.setTimedServoPosition(servo3, 1, 10000);
        while (true){
            servoMgr.updateServos();
        }
    }
}
