package org.firstinspires.ftc.teamcode.previousCode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class PsitionTest extends LinearOpMode {

    RobotAuto robotAuto;

    @Override
    public void runOpMode()  {

        robotAuto = new RobotAuto(this);

        waitForStart();

        while(opModeIsActive()) {
            SparkFunOTOS.Pose2D a = robotAuto.getPosition();

            telemetry.addData("x", a.x);
            telemetry.addData("y", a.y);
            telemetry.addData("h", a.h);
            telemetry.update();
        }
    }
}
