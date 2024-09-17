package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ManualOpModeTest extends LinearOpMode
{

    DcMotor motor;
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        float x = gamepad1.left_stick_x;
        if ( gamepad1.left_stick_x > 0) {

            motor.setPower(x);

        }
    }
}
