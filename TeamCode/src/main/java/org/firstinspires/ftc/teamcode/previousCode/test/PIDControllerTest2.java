package org.firstinspires.ftc.teamcode.previousCode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@Disabled
@TeleOp(group = "Test")
public class PIDControllerTest2 extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotor motor = hardwareMap.get(DcMotor.class, "motor");
        waitForStart();
        Gamepad previousGamepad1 = new Gamepad();
        previousGamepad1.copy(gamepad1);
        double currentPos = 0, previousPos = 0, offset = 0;
        double targetPos = 0;
        double ticks_per_rev = 530;
        double delta = 0;
        double p = 0, i = 0 , d = 0;
        final double Kp = 0.02, Ki = 0.00008, Kd = -0.01, power_max = 1;
        double power = 0;
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive()){
            previousPos = currentPos;
            currentPos = motor.getCurrentPosition();
            delta = targetPos - currentPos - offset;
            if (delta > ticks_per_rev * 0.75)
                power = 1;
            else if (delta < -ticks_per_rev * 0.75)
                power = -1;
            else
            {
                p = Kp * delta;
                i += Ki * delta;
                d = Kd * (currentPos - previousPos);
                power = p + i + d;
                if (power > power_max)
                    power = power_max;
                else if(power < -power_max)
                    power = -power_max;
            }
            motor.setPower(power);
            previousGamepad1.copy(gamepad1);
            telemetry.addData("power",power);
            telemetry.addData("p",p);
            telemetry.addData("i",i);
            telemetry.addData("d",d);
            telemetry.addData("tpr",ticks_per_rev);
            telemetry.update();
            sleep(10);
            if(gamepad1.a) {
                targetPos = 0;
            }
            if(gamepad1.b){
                targetPos = ticks_per_rev;
            }
            if(gamepad1.start){
                offset = motor.getCurrentPosition();
                p = i = d = 0;
            }
        }
    }
}