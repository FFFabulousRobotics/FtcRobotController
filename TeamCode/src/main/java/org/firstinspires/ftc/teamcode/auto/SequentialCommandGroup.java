package org.firstinspires.ftc.teamcode.auto;

public class SequentialCommandGroup implements Command{
    Command[] commandGroup;
    int cmdLength;
    int currentIndex = 0;

    public SequentialCommandGroup(Command... commands) {
        this.commandGroup = commands;
        this.cmdLength = commandGroup.length;
    }

    @Override
    public void init(){
        commandGroup[0].init();
    }

    @Override
    public void finish(){
        commandGroup[cmdLength-1].finish();
    }

    @Override
    public void iterate() {
        if(currentIndex >= cmdLength)return;

        Command currentCommand = commandGroup[currentIndex];
        if(currentCommand.isContinuous()){
            currentCommand.iterate();
        }else {
            if (currentCommand.hasNext()) {
                currentCommand.iterate();
            } else {
                currentCommand.finish();
                currentIndex += 1;
                if (currentIndex < cmdLength) commandGroup[currentIndex].init();
            }
        }
    }

    @Override
    public boolean hasNext() {
        if(currentIndex == cmdLength - 1){
            return commandGroup[currentIndex].hasNext();
        } else return currentIndex < cmdLength;
    }

    public void runCommand(){
        init();
        while (hasNext()){
            iterate();
        }
        finish();
    }

    @Override
    public boolean isContinuous(){
        for (Command eachCommand: commandGroup) {
            if(eachCommand.isContinuous())return true;
        }
        return false;
    }
}
