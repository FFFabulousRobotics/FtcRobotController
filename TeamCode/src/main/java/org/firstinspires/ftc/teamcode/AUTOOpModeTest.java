package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AUTOOpModeTest extends LinearOpMode
{

    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException
    {

        motor = hardwareMap.dcMotor.get("LeftRightmotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        motor.setPower(1);
        motor.setTargetPosition(1000);

    }
}