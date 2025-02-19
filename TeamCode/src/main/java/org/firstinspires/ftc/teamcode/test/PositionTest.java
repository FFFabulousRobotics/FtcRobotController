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
        GoBildaPinpointDriver odo;
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        waitForStart();
        while(opModeIsActive()) {

            telemetry.addData("x", odo.getPosX());
            telemetry.addData("y", odo.getPosY());
            telemetry.addData("h", odo.getHeading());
            telemetry.update();
        }
    }
}
