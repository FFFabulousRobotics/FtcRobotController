package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class ServoTest extends LinearOpMode {

    private Servo servo1;
    private double position = 0.5;
    private final double step = 0.01;

    @Override
    public void runOpMode() {
        // 初始化舵机
        servo1 = hardwareMap.get(Servo.class, "servo1");

        waitForStart();

        while (opModeIsActive()) {
            // 控制舵机位置
            if (gamepad1.a && position + step <= 1.0) {

                position += step;
                servo1.setPosition(position);
                sleep(50);
            } else if (gamepad1.b && position - step >= 0) {

                position -= step;
                servo1.setPosition(position);
                sleep(50);
            }

            // 添加其他舵机控制逻辑
            telemetry.addData("Servo Position", servo1.getPosition());
            telemetry.update();
        }
    }
}
