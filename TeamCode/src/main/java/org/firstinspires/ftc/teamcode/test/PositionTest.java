package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;


@TeleOp
public class PositionTest extends LinearOpMode {

    RobotAuto robotAuto;

    @Override
    public void runOpMode()  {

        robotAuto = new RobotAuto(this);

        waitForStart();
        while(opModeIsActive()) {

            robotAuto.spin(180);
            telemetry.addData("x", robotAuto.getPosition().x);
            telemetry.addData("y", robotAuto.getPosition().y);
            telemetry.addData("h", robotAuto.getPosition().h);
            telemetry.update();
        }
    }
}
