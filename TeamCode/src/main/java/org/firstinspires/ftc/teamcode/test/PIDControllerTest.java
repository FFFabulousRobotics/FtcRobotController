package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.RobotTop;

@TeleOp
public class PIDControllerTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        RobotTop robotTop = new RobotTop(this);

        waitForStart();
        Gamepad previousGamepad1 = new Gamepad();
        previousGamepad1.copy(gamepad1);
        double currentPos = 0, previousPos;
        double targetPos = 1000;
        double delta;
        double p = 0, i = 0 , d = 0;
        final double Kp = 0.1, Ki = 0.0, Kd = 0.0;
        double power;

        while (opModeIsActive()){
            robotTop.setTopServoPosition(0.05);
            if(gamepad1.a) {
                previousPos = currentPos;
                currentPos = robotTop.getLiftPosition();
                delta = targetPos - currentPos;
                p = Kp * delta;
                i += Ki * delta;
                d = Kd * (currentPos - previousPos);
                power = Math.min(p + i + d, 1);
                robotTop.setLeftPower(power);
                previousGamepad1.copy(gamepad1);
                telemetry.addData("power",power);
                telemetry.addData("p",p);
                telemetry.addData("i",i);
                telemetry.addData("d",d);
                telemetry.update();
                sleep(10);
            }else {
                robotTop.setLeftPower(0);
            }
        }
    }
}
