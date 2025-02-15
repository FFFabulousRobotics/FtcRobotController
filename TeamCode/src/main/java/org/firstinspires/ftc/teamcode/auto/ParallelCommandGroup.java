package org.firstinspires.ftc.teamcode.auto;

import java.util.Arrays;

public class ParallelCommandGroup implements Command {
    Command[] commandGroup;
    boolean[] isFinished;

    public ParallelCommandGroup(Command... commands) {
        this.commandGroup = commands;
        isFinished = new boolean[commands.length];
        Arrays.fill(isFinished, false);
    }

    @Override
    public void iterate() {
        for (int i = 0; i < commandGroup.length; i++) {
            Command eachCommand = commandGroup[i];
            boolean eachCmdFinished = isFinished[i];

            if(eachCommand.isContinuous()){
                eachCommand.iterate();
                isFinished[i] = eachCommand.hasNext();
            }
            if(!eachCmdFinished) {
                if (eachCommand.hasNext()) {
                    eachCommand.iterate();
                } else {
                    eachCommand.finish();
                    isFinished[i] = true;
                }
            }
        }
    }

    @Override
    public boolean hasNext() {
        for(boolean each: isFinished){
            if(!each){
                return true;
            }
        }
        return false;
    }

    @Override
    public void init() {
        for (Command eachCommand : commandGroup) {
            eachCommand.init();
        }
    }

    @Override
    public void finish() {
        Command.super.finish();
    }

    @Override
    public void cancel() {
        Command.super.cancel();
    }

    public void runCommand(){
        init();
        while (hasNext()){
            iterate();
        }
        finish();
    }
}
