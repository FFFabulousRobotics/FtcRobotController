package org.firstinspires.ftc.teamcode.auto;

public abstract class ContinuousCommand implements Command {
    public boolean isContinuous = true;

    public abstract void iterate();

    public abstract boolean hasNext();

    public abstract void init();

    @Override
    public void cancel() {
        Command.super.cancel();
    }

    @Override
    public boolean isContinuous() {
        return true;
    }
}
