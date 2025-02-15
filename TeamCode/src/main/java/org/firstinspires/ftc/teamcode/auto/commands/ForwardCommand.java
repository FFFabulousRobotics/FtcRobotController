package org.firstinspires.ftc.teamcode.auto.commands;

import org.firstinspires.ftc.teamcode.auto.Command;

public class ForwardCommand implements Command {
    double distance;
    double maxDriveSpeed = 0.7;

    public ForwardCommand(double distance){

    }
    @Override
    public void iterate() {

    }

    @Override
    public boolean hasNext() {
        return false;
    }

    @Override
    public void init() {
        Command.super.init();
    }

    @Override
    public void finish() {
        Command.super.finish();
    }
}
