package org.firstinspires.ftc.teamcode.previousCode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(group = "Test")
public class LeftTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotor leftLiftMotor = hardwareMap.get(DcMotor.class , "leftLift");
        DcMotor rightLiftMotor = hardwareMap.get(DcMotor.class, "rightLift");

        leftLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightLiftMotor.setDirection(DcMotor.Direction.REVERSE);

        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        if (isStopRequested()) return;
        double liftSpeed = 0;
        double pos = 0;

        while (opModeIsActive()) {
            pos = leftLiftMotor.getCurrentPosition();
            boolean y = gamepad1.y;
            boolean a = gamepad1.a;
            if(y && pos <= 1200){
                liftSpeed =0.5;
            } else if (a && pos >= 100) {
                liftSpeed = -0.5;
            }else{liftSpeed = 0;}
            leftLiftMotor.setPower(liftSpeed);
            rightLiftMotor.setPower(liftSpeed);
            telemetry.addData("pos",leftLiftMotor.getCurrentPosition());
            telemetry.update();
            sleep(10);
        }
        //[-219, 1071]
    }
}
