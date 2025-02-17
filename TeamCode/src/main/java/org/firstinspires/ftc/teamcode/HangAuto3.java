package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.auto.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.auto.commands.ForwardCommand;
import org.firstinspires.ftc.teamcode.auto.commands.GotoPosCommand;
import org.firstinspires.ftc.teamcode.auto.commands.GotoPosWithHeadingCommand;
import org.firstinspires.ftc.teamcode.auto.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.auto.commands.SetLiftPositionCommand;
import org.firstinspires.ftc.teamcode.auto.commands.SleepCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;

@Autonomous
public class HangAuto3 extends LinearOpMode {
    final int STRETCH_BACK_POSITION = 70;
    final int STRETCH_OUT_POSITION = 1500;
    final double SPIN_DEFAULT_POSITION_L = 1;
    final double SPIN_DEFAULT_POSITION_R = 0.464;
    final double SPIN_HOVERING_POSITION_L = 0.4;
    final double SPIN_HOVERING_POSITION_R = 1;
    final double SPIN_DOWN_POSITION = 0;
    final double TURN_BACK_POSITION = 0.5;
    final double TURN_LOCK_POSITION = 0.68;
    final double TURN_HOVERING_POSITION = 0.75;
    final double TURN_DOWN_POSITION = 0.85;
    final double GRAB_OPEN_POSITION = 0.4;
    final double GRAB_CLOSE_POSITION = 0.92;
    final double TOP_BACK = 0.03;
    final double TOP_OUT = 0.66;
    final double BACK_CLOSE = 0.7;
    final double BACK_OPEN = 0.1;

    final private double[] posHanging = {10, -22, 0};//挂的地方
    final private double[] posHanging2 = {5, -21.5, 0};//第二次挂的地方(有误差)
    final private double[] posHanging3 = {2, -20.9, 0};//第三次挂的地方(有误差)
    final private double[] posMiddleStop = {-20, -10, 0};//中间停靠点
    final private double[] posReadyForPush1 = {-24, -43, 90};//准备推的位置
    final private double[] posPushed = {-29, -4, 90};//推到这个位置
    final private double[] posGet = {-27,-2,180};//拿方块的地方

    RobotTop robotTop;
    RobotAuto robotAuto;

    @Override
    public void runOpMode(){
        robotTop = new RobotTop(this);
        robotAuto = new RobotAuto(this);
        waitForStart();

        robotTop.setArmLeftSpinPosition(SPIN_DEFAULT_POSITION_L);
        robotTop.setArmRightSpinPosition(SPIN_DEFAULT_POSITION_R);
        robotTop.setLiftServoPosition(BACK_CLOSE);

        hangSample1();
        pushSample1();
        hangSample2();
        hangSample3();
        hangSample4();
        getSample2();
        getSample3();
    }

    protected void hangSample1(){
        ParallelCommandGroup cmd1 = new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new GotoPosWithHeadingCommand(robotAuto,posHanging[0],posHanging[1],posHanging[2])
                ),
                new SequentialCommandGroup(
                        new InstantCommand(() -> robotTop.setTurnPosition(TURN_LOCK_POSITION)),
                        new SetLiftPositionCommand(robotAuto,920)
                )
        );
        cmd1.runCommand();
        SequentialCommandGroup cmd2 = new SequentialCommandGroup(
                new InstantCommand(() -> robotTop.setLiftPower(-0.2)),
                new SleepCommand(700),
                new InstantCommand(() -> robotTop.setLiftPower(0)),
                new InstantCommand(robotAuto::release)
        );
        cmd2.runCommand();
    }

    protected void pushSample1(){
        SequentialCommandGroup cmd3 = new SequentialCommandGroup(
                new GotoPosCommand(robotAuto,posMiddleStop[0],posMiddleStop[1]),
                new GotoPosWithHeadingCommand(robotAuto, posReadyForPush1[0], posReadyForPush1[1], posReadyForPush1[2]),
                new GotoPosWithHeadingCommand(robotAuto,posPushed[0],posPushed[1],posPushed[2]),
                new GotoPosWithHeadingCommand(robotAuto, posGet[0], posGet[1], posGet[2]),
                new ForwardCommand(robotAuto,-10),
                new InstantCommand(robotAuto::grab),
                new ForwardCommand(robotAuto,1)
        );
        cmd3.runCommand();
    }

    protected void hangSample2(){
        ParallelCommandGroup cmd1 = new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new GotoPosWithHeadingCommand(robotAuto,posHanging2[0],posHanging2[1],posHanging2[2])
                ),
                new SequentialCommandGroup(
                        new InstantCommand(() -> robotTop.setTurnPosition(TURN_LOCK_POSITION)),
                        new SetLiftPositionCommand(robotAuto,920)
                )
        );
        cmd1.runCommand();

        SequentialCommandGroup cmd2 = new SequentialCommandGroup(
                new InstantCommand(() -> robotTop.setLiftPower(-0.2)),
                new SleepCommand(700),
                new InstantCommand(() -> robotTop.setLiftPower(0)),
                new InstantCommand(robotAuto::release)
        );
        cmd2.runCommand();
    }

    protected void getSample2(){
        SequentialCommandGroup cmd5 = new SequentialCommandGroup(
                new InstantCommand(() -> robotTop.setLiftPower(0)),
                new GotoPosWithHeadingCommand(robotAuto, posGet[0], posGet[1], posGet[2]),
                new ForwardCommand(robotAuto,-10),
                new InstantCommand(robotAuto::grab),
                new ForwardCommand(robotAuto,1)
        );
        cmd5.runCommand();
    }

    protected void hangSample3(){
        ParallelCommandGroup cmd1 = new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new GotoPosWithHeadingCommand(robotAuto,posHanging3[0],posHanging3[1],posHanging3[2])
                ),
                new SequentialCommandGroup(
                        new InstantCommand(() -> robotTop.setTurnPosition(TURN_LOCK_POSITION)),
                        new SetLiftPositionCommand(robotAuto,920)
                )
        );
        cmd1.runCommand();

        SequentialCommandGroup cmd2 = new SequentialCommandGroup(
                new InstantCommand(() -> robotTop.setLiftPower(-0.2)),
                new SleepCommand(700),
                new InstantCommand(() -> robotTop.setLiftPower(0)),
                new InstantCommand(robotAuto::release)
        );
        cmd2.runCommand();
    }

    protected void getSample3(){
        SequentialCommandGroup cmd5 = new SequentialCommandGroup(
                new InstantCommand(() -> robotTop.setLiftPower(0)),
                new GotoPosWithHeadingCommand(robotAuto, posGet[0], posGet[1], posGet[2]),
                new ForwardCommand(robotAuto,-10),
                new InstantCommand(robotAuto::grab),
                new ForwardCommand(robotAuto,1)
        );
        cmd5.runCommand();
    }

    protected void hangSample4(){
        ParallelCommandGroup cmd1 = new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new GotoPosWithHeadingCommand(robotAuto,posHanging3[0],posHanging3[1],posHanging3[2])
                ),
                new SequentialCommandGroup(
                        new InstantCommand(() -> robotTop.setTurnPosition(TURN_LOCK_POSITION)),
                        new SetLiftPositionCommand(robotAuto,920)
                )
        );
        cmd1.runCommand();

        SequentialCommandGroup cmd2 = new SequentialCommandGroup(
                new InstantCommand(() -> robotTop.setLiftPower(-0.2)),
                new SleepCommand(700),
                new InstantCommand(() -> robotTop.setLiftPower(0)),
                new InstantCommand(robotAuto::release)
        );
        cmd2.runCommand();
    }

}