package org.firstinspires.ftc.teamcode.previousCode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "Test")
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        Servo armServo0 = hardwareMap.get(Servo.class , "arm0");
        Servo armServo1 = hardwareMap.get(Servo.class , "arm1");
        //stretch(1): [0, 0.3]
        //turn(3): [0.38-back, 0.71-out]
        //grab(5): [0-open, 0.53-close]
        //spinY(4): default: 0.07
        //spinX(2): default: 0.5

        waitForStart();
        Gamepad previousGamepad1 = new Gamepad();
        previousGamepad1.copy(gamepad1);
        double servoPos = 0.5;

        while (opModeIsActive()){
            boolean x = gamepad1.x;
            boolean b = gamepad1.b;
            if(x){
                servoPos = Math.min(1, servoPos + 0.003);
            }if(b){
                servoPos = Math.max(0, servoPos - 0.003);
            }
            armServo0.setPosition(0.5-servoPos);
            armServo1.setPosition(0.5+servoPos);
            telemetry.addData("pos", servoPos);
            telemetry.update();

            previousGamepad1.copy(gamepad1);
            sleep(10);
        }
    }
}
