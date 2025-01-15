package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.RobotTop;

@TeleOp
public class PIDControllerTest2 extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotor motor = hardwareMap.get(DcMotor.class, "motor");
        waitForStart();
        Gamepad previousGamepad1 = new Gamepad();
        previousGamepad1.copy(gamepad1);
        double currentPos = 0, previousPos;
        double targetPos = 300;
        double delta;
        double p = 0, i = 0 , d = 0;
        final double Kp = 0.02, Ki = 0.00007, Kd = -0.008, power_max = 1;
        double power;

        while (opModeIsActive()){
            previousPos = currentPos;
            currentPos = motor.getCurrentPosition();
            delta = targetPos - currentPos;
            p = Kp * delta;
            i += Ki * delta;
            d = Kd * (currentPos - previousPos);
            power = p + i + d;
            if (power > power_max)
                power = power_max;
            else if(power < -power_max)
                power = -power_max;
            motor.setPower(power);
            previousGamepad1.copy(gamepad1);
            telemetry.addData("power",power);
            telemetry.addData("p",p);
            telemetry.addData("i",i);
            telemetry.addData("d",d);
            telemetry.update();
            sleep(10);
            if(gamepad1.a) {
                targetPos += 50;
            }
            if(gamepad1.b){
                targetPos -= 50;
            }
        }
    }
}
