package org.firstinspires.ftc.teamcode;

public class TaskStatus {
    int delayTicks;
    int loopTicks;
    int remainingLoopTicks;
    int afterTaskId;

    boolean idling = true;

    public TaskStatus(int delayTicks, int loopTicks, int afterTaskId) {
        this.delayTicks = delayTicks;
        this.loopTicks = loopTicks;
        this.afterTaskId = afterTaskId;
        this.remainingLoopTicks = loopTicks;
    }

    public boolean tickDown() {
        if (idling) {
            delayTicks--;
            if (delayTicks == 0) {
                idling = false;
                return true;
            }
        } else {
            remainingLoopTicks--;
            if (remainingLoopTicks == 0) {
                remainingLoopTicks = loopTicks;
                return true;
            }
        }
        return false;
    }
}
