package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(group = "Test")
public class Armtest extends LinearOpMode {
    @Override
    public void runOpMode(){
        Servo armServo0 = hardwareMap.get(Servo.class , "arm0");
        Servo armServo1 = hardwareMap.get(Servo.class , "arm1");
        DcMotor m = hardwareMap.get(DcMotor.class, "m");

        waitForStart();
        Gamepad previousGamepad1 = new Gamepad();
        previousGamepad1.copy(gamepad1);

        if (isStopRequested()) return;
        double leftPos = 0.5;
        double rightPos = 0.5;

        while (opModeIsActive()) {
            if(gamepad1.dpad_up){
                leftPos = Math.min(1, leftPos + 0.003);
                rightPos = Math.min(1, rightPos + 0.003);
            }
            if(gamepad1.dpad_down){
                leftPos = Math.max(0, leftPos - 0.003);
                rightPos = Math.max(0, rightPos - 0.003);
            }
            if(gamepad1.dpad_left){
                leftPos = Math.min(1, leftPos + 0.003);
                rightPos = Math.max(0, rightPos - 0.003);
            }
            if(gamepad1.dpad_right){
                leftPos = Math.max(0, leftPos - 0.003);
                rightPos = Math.min(1, rightPos + 0.003);
            }

            if(gamepad1.y){
                leftPos = Math.min(1, leftPos + 0.003);
                rightPos = Math.max(0, rightPos - 0.003);
            }
            if(gamepad1.a){
                leftPos = Math.max(0, leftPos - 0.003);
                rightPos = Math.min(1, rightPos + 0.003);
            }
            if(gamepad1.x){
                m.setPower(-0.3);
            } else if (gamepad1.b) {
                m.setPower(0.3);
            }else {
                m.setPower(0);
            }
            armServo0.setPosition(leftPos);
            armServo1.setPosition(rightPos);

            telemetry.addData("lp", leftPos);
            telemetry.addData("pos", m.getCurrentPosition());
            telemetry.update();

            sleep(10);
        }
    }
}

