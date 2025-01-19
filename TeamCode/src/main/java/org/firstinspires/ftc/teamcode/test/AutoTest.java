package org.firstinspires.ftc.teamcode.test;

import com.qualcomm. robotcore.eventloop. opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm. robotcore. eventloop. opmode. LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm. robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous(group = "Test")
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "FL");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "BL");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "FR");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "BR");
        DcMotor leftLift = hardwareMap.get(DcMotor.class,"leftLift");
        DcMotor rightLift = hardwareMap.get(DcMotor.class,"rightLift");
        DcMotor armStretchMotor = hardwareMap.get(DcMotor.class, "armStretch");
        Servo liftServo = hardwareMap.get(Servo.class,"liftServo");

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armStretchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.REVERSE);
        armStretchMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        //tighten
        liftServo.setPosition(0.95);
        sleep(700);

        frontLeft.setPower(-0.7);
        backLeft.setPower(0.7);
        frontRight.setPower(0.7);
        backRight.setPower(-0.7);
        sleep(700);

        //forward
        frontLeft.setPower(0.7);
        backLeft.setPower(0.7);
        frontRight.setPower(0.7);
        backRight.setPower(0.7);
        sleep(600);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        //stretch arm
        armStretchMotor.setPower(0.5);
        sleep(1400);
        armStretchMotor.setPower(0);

        //lift
        leftLift.setPower(0.5);
        rightLift.setPower(0.5);
        sleep(1700);
        leftLift.setPower(0);
        rightLift.setPower(0);

        frontLeft.setPower(0.5);
        backLeft.setPower(0.5);
        frontRight.setPower(0.5);
        backRight.setPower(0.5);
        sleep(130);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        leftLift.setPower(-0.5);
        rightLift.setPower(-0.5);
        while(leftLift.getCurrentPosition() < 800);
        leftLift.setPower(0);
        rightLift.setPower(0);

        //Release
        liftServo.setPosition(0.2);
        sleep(500);

        armStretchMotor.setPower(-0.5);
        sleep(1400);
        armStretchMotor.setPower(0);

        //back1
        frontLeft.setPower(-0.7);
        backLeft.setPower(-0.7);
        frontRight.setPower(-0.7);
        backRight.setPower(-0.7);
        sleep(500);

        //rafe
        frontLeft.setPower(0.7);
        backLeft.setPower(-0.7);
        frontRight.setPower(-0.7);
        backRight.setPower(0.7);
        sleep(1500);

        //spin
        frontLeft.setPower(-0.7);
        backLeft.setPower(-0.7);
        frontRight.setPower(0.7);
        backRight.setPower(0.7);
        sleep(1320);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        //back2
        frontLeft.setPower(0.7);
        backLeft.setPower(0.7);
        frontRight.setPower(0.7);
        backRight.setPower(0.7);
        sleep(100);

        liftServo.setPosition(0.95);
        sleep(700);

    }
}