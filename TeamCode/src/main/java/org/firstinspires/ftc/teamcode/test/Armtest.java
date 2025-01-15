package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "Test")
public class Armtest extends LinearOpMode {
    @Override
    public void runOpMode(){
        Servo armServo0 = hardwareMap.get(Servo.class , "arm0");
        Servo armServo1 = hardwareMap.get(Servo.class , "arm1");

        waitForStart();
        Gamepad previousGamepad1 = new Gamepad();
        previousGamepad1.copy(gamepad1);

        if (isStopRequested()) return;
        double leftPos = 0;
        double rightPos = 0;

        while (opModeIsActive()) {
            if(gamepad1.y){
                leftPos = Math.min(1, leftPos + 0.003);
                rightPos = Math.min(1, rightPos + 0.003);
            }
            if(gamepad1.a){
                leftPos = Math.max(0, leftPos - 0.003);
                rightPos = Math.max(0, rightPos - 0.003);
            }
            if(gamepad1.x){
                leftPos = Math.min(1, leftPos + 0.003);
                rightPos = Math.max(0, rightPos - 0.003);
            }
            if(gamepad1.b){
                leftPos = Math.max(0, leftPos - 0.003);
                rightPos = Math.min(1, rightPos + 0.003);
            }
            armServo0.setPosition(leftPos);
            armServo1.setPosition(rightPos);

            sleep(10);
        }
    }
}

