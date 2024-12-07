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
        robotAuto.fastForward(96)
                .sleep(1000)
                .leftShift(-12)
                .sleep(30000);
        telemetry.addData("x",robotAuto.getPosition().x);
        telemetry.addData("y",robotAuto.getPosition().y);
        telemetry.addData("h",robotAuto.getPosition().h);
        telemetry.update();
        robotAuto.grab().setLiftPower(0.5).sleep(400).fastForward(24).rightShift(48).fastSpin(180)
                .setLiftPower(0.3).sleep(500).backward(10).setLiftPower(-0.5).sleep(200).setLiftPower(0);
    }
}
