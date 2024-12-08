package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

//It proves out to be useless.
//Never use Thread in the Robot!

@Disabled
public class ThreadTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        Servo armServo = hardwareMap.get(Servo.class , "arm");
        waitForStart();
        Gamepad previousGamepad1 = new Gamepad();
        previousGamepad1.copy(gamepad1);
        double a = 0;
        Thread t = new Thread(){
            @Override
            public void run() {
                super.run();
                try {
                    telemetry.addData("q", 1);
                    telemetry.update();
                    armServo.setPosition(0.9);
                    sleep(3000);
                    armServo.setPosition(0.3);
                    sleep(3000);
                    armServo.setPosition(0.9);
                    sleep(3000);
                    armServo.setPosition(0.3);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        };
        t.start();
        while (opModeIsActive()){
            a += 1;
            telemetry.addData("time", a);
            telemetry.update();
            if(gamepad1.y && !previousGamepad1.y) {
                Thread th = timedSetPosition(armServo, 0.2, 2000);
                th.start();
            }
            sleep(1000);
            previousGamepad1.copy(gamepad1);
        }
    }
    public Thread timedSetPosition(Servo servo, double position, double millisecond){
        telemetry.addData("going to run",1);
        return new Thread(){
            @Override
            public void run() {
                final int TICK_MS = 50;
                double deltaPos = position - servo.getPosition();
                double iterationCount = millisecond / TICK_MS;
                double perPos = deltaPos / iterationCount;
                telemetry.addData("running", 1);
                try{
                    super.start();
                    double servoPos = servo.getPosition();
                    for(; iterationCount <= 0; iterationCount--){
                        servoPos += perPos;
                        servo.setPosition(servoPos);
                        sleep(TICK_MS);
                        telemetry.addData("for_repetition", 1);
                    }
                }catch (InterruptedException e){
                    throw new RuntimeException(e);
                }
            }
        };
    }
}
