package org.firstinspires.ftc.teamcode.advancedManual;

import android.util.ArrayMap;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class ServoManager {
    protected Map<Integer, ServoTask> tasks;

    public ServoManager() {
        tasks = new ArrayMap<>();
    }

    public void updateServos(){
        for (Map.Entry<Integer, ServoTask> taskEntry : tasks.entrySet()) {
            int taskId = taskEntry.getKey();
            ServoTask task = taskEntry.getValue();
            if (!task.hasNext()) {
                task.finish();
                tasks.remove(taskId, task);
                continue;
            }
            task.iterate();
        }
    }

    /**
     * Sets the current position of the servo in a fixed time, expressed as a fraction of
     * its available range. If PWM power is enabled for the servo, the servo will attempt
     * to move to the indicated position.
     *
     * @param position the position to which the servo should move, a value in the range [0.0, 1.0]
     * @param timeMs   the time that the servo will take to move.
     * @return a Task object
     */
    private ServoTask timedSetPosition(Servo servo, double position, long timeMs) {
        final double deltaPos = position - servo.getPosition();
        final long iterationCount = timeMs / ServoTask.TICK_MS;
        final double positionPerIteration = deltaPos / iterationCount;
        return new ServoTask() {
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
    public void setTimedServoPosition(Servo servo, double position, long timeMs){
        ServoTask task = timedSetPosition(servo, position, timeMs);
        int taskId = findMinFreeTaskId();
        tasks.put(taskId, task);
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