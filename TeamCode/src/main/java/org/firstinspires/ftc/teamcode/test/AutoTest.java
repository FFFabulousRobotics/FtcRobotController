package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.hardware.RobotChassis;

@Autonomous
public class AutoTest extends LinearOpMode {
    RobotAuto robotAuto;
    RobotChassis robotChassis;

    @Override
    public void runOpMode() {
        robotChassis = new RobotChassis(this);
        robotAuto = new RobotAuto(this);
        waitForStart();
        resetRuntime();
        robotAuto.fastForward(24);
    }
}