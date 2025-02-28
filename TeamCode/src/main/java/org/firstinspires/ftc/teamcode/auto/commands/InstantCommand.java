package org.firstinspires.ftc.teamcode.auto.commands;

import org.firstinspires.ftc.teamcode.auto.Command;

public class InstantCommand implements Command {
    Runnable fun;
    public InstantCommand(Runnable fun){
        this.fun = fun;
    }

    @Override
    public void iterate(){}

    @Override
    public boolean hasNext() {
        return false;
    }

    @Override
    public void init() {
        fun.run();
    }

    @Override
    public void finish() {
        Command.super.finish();
    }

    @Override
    public void cancel() {
        Command.super.cancel();
    }
}
