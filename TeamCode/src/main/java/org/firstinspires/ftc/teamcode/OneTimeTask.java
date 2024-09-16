package org.firstinspires.ftc.teamcode;

public abstract class OneTimeTask implements Task {
    @Override
    public void iterate() {}

    @Override
    public boolean hasNext() {
        return false;
    }
}
