package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotAuto;

@Autonomous
public class RedLeftAuto extends LinearOpMode {
    RobotAuto robotAuto;
    @Override
    public void runOpMode(){
        robotAuto = new RobotAuto(this);
        waitForStart();
        resetRuntime();

        robotAuto.grab()
                .setLiftPower(0.5)
                .sleep(400)
                .setLiftPower(0)
                .fastBackward(24)
                .leftShift(24)
                .fastSpin(180)
                .setLiftPower(0.3)
                .sleep(800)
                .fastBackward(10)
                .setLiftPower(-0.5)
                .sleep(200)
                .setLiftPower(0)
                .release();
    }
}
