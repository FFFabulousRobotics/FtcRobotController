package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.auto.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.auto.commands.GotoPosWithHeadingCommand;
import org.firstinspires.ftc.teamcode.auto.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.auto.commands.SetLiftPositionCommand;
import org.firstinspires.ftc.teamcode.auto.commands.SleepCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;

@Autonomous
public class BasketballAuto extends LinearOpMode {
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

    final private double[] posBasket = {-36.5, 3.5, -43.8135};//投篮位置
    final private double[] posGrab1 = {-26, 16.4898, 10.5908};//第1个夹取位置
    final private double[] posGrab2 = {-30.3891, 18.8595, 28.0001};//第1个夹取位置
    final private double[] posGrab3 = {-29, 20, 45};
    final private double[] parkingPos = {-8,45.8,90.9};
//    final private double posT[][] = {
//            {-25, 16.4898, 10.5908},//第1个夹取位置
//            {-29.3891, 18.8595, 28.0001},//第2个夹取位置
//            {-25.9, 16.4898, 10.5908},//第1个夹取位置
//            {-29.3891, 15.8595, 22.0001}};//停靠位置
    RobotTop robotTop;
    RobotAuto robotAuto;

    @Override
    public void runOpMode(){
        robotTop = new RobotTop(this);
        robotAuto = new RobotAuto(this);

        waitForStart();
        robotTop.setTurnPosition(TURN_LOCK_POSITION);
        robotTop.setArmLeftSpinPosition(SPIN_DEFAULT_POSITION_L);
        robotTop.setArmRightSpinPosition(SPIN_DEFAULT_POSITION_R);
        robotTop.setArmGrabPosition(GRAB_OPEN_POSITION);

        //main movement
        basketball();
        pickupSample(posGrab1);
        basketball();
        pickupSample(posGrab2);
        basketball();
        pickupSample(posGrab3);
        basketball();
        parking();

        telemetry.addData("3",3);
        telemetry.update();
    }

    protected void basketball(){
        ParallelCommandGroup cmd1 = new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robotTop.setTurnPosition(TURN_LOCK_POSITION)),
                        new GotoPosWithHeadingCommand(robotAuto,posBasket[0], posBasket[1], posBasket[2]),
                        new InstantCommand(robotAuto::topOut),
                        new SleepCommand(800),
                        new InstantCommand(robotAuto::topBack)),
                new SequentialCommandGroup(
                        new SleepCommand(300),
                        new SetLiftPositionCommand(robotAuto,1260)
                )
        );
        cmd1.runCommand();
    }

    protected void pickupSample(double[] pos){
        ParallelCommandGroup cmd2 = new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new GotoPosWithHeadingCommand(robotAuto,pos[0], pos[1], pos[2]),
                        new InstantCommand(robotAuto::armHover),
                        new SleepCommand(500),
                        new InstantCommand(robotAuto::armDown),
                        new SleepCommand(200),
                        new InstantCommand(robotAuto::armGrab),
                        new SleepCommand(300)
                ),
                new SequentialCommandGroup(
                        new SleepCommand(800),
                        new SetLiftPositionCommand(robotAuto,70)
                )

        );
        cmd2.runCommand();

        SequentialCommandGroup cmd3 = new SequentialCommandGroup(
                new InstantCommand(() -> robotTop.setLiftPower(0)),
                new InstantCommand(robotAuto::armBack),
                new SleepCommand(500),
                new InstantCommand(robotAuto::armRelease),
                new SleepCommand(400),
                new InstantCommand(() -> robotTop.setTurnPosition(TURN_LOCK_POSITION))
        );
        cmd3.runCommand();
    }

    protected void parking(){
        ParallelCommandGroup cmd4 = new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new GotoPosWithHeadingCommand(robotAuto,parkingPos[0],parkingPos[1],parkingPos[2]),
                        new InstantCommand(robotAuto::topOut)
                        ),
                new SetLiftPositionCommand(robotAuto,100)
        );
        cmd4.runCommand();
        robotAuto.fastBackward(10);
    }
}