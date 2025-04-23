package org.firstinspires.ftc.teamcode.auto.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.Command;

public class SleepCommand implements Command {
    ElapsedTime timer = new ElapsedTime();
    int sleepTime;
    public SleepCommand(int millisecond){
        sleepTime = millisecond;
    }

    @Override
    public void iterate() {
    }

    @Override
    public boolean hasNext() {
        return timer.milliseconds() <= sleepTime;
    }

    @Override
    public void init() {
        timer.reset();
    }
}
