package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "Test")
public class tttest extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotor m = hardwareMap.get(DcMotor.class, "m");

        waitForStart();
        Gamepad previousGamepad1 = new Gamepad();
        previousGamepad1.copy(gamepad1);

        if (isStopRequested()) return;
        m.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()) {
            if(gamepad1.y){
                m.setTargetPosition(0);
            }
            if(gamepad1.a){
                m.setTargetPosition(300);
            }

            // telemetry.addData("lp", leftPos);
            telemetry.update();

            sleep(10);
        }
    }
}
