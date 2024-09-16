package org.firstinspires.ftc.teamcode;

public class SleepTask implements Task {
    int remainingSleepTicks;

    public SleepTask(int sleepTicks) {
        remainingSleepTicks = sleepTicks;
    }

    @Override
    public void iterate() {
        remainingSleepTicks--;
    }

    @Override
    public boolean hasNext() {
        return remainingSleepTicks > 0;
    }
}
