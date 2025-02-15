package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.auto.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.auto.commands.SleepCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;

@Autonomous(group = "Test")
public class CommandTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        RobotTop robotTop = new RobotTop(this);
        waitForStart();
        ParallelCommandGroup cmd = new ParallelCommandGroup(
                new SleepCommand(300),
                new InstantCommand(() -> robotTop.setLiftPower(0.1))
        );
        cmd.runCommand();
    }
}