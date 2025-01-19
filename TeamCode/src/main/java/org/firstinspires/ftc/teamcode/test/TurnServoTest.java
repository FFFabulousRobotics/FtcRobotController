package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(group = "Test")
public class TurnServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo armLeftTurnServo = hardwareMap.get(Servo.class, "armTurnL");
        Servo armRightTurnServo = hardwareMap.get(Servo.class, "armTurnR");

        double armPos = 0.0; // left

        boolean pU = gamepad1.dpad_up;
        boolean pD = gamepad1.dpad_down;

        waitForStart();

        while (opModeIsActive()) {
            if (!pU && gamepad1.dpad_up) {
                armPos += 0.01;
            } else if (!pD && gamepad1.dpad_down) {
                armPos -= 0.01;
            }

            armLeftTurnServo.setPosition(armPos);
            armRightTurnServo.setPosition(1 - armPos);

            pU = gamepad1.dpad_up;
            pD = gamepad1.dpad_down;

            sleep(10);
        }
    }
}
