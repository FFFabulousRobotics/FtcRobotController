package org.firstinspires.ftc.teamcode.test;

import com.qualcomm. robotcore.eventloop. opmode.Autonomous;
import com.qualcomm. robotcore. eventloop. opmode. LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;

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
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.REVERSE);
        armStretchMotor.setDirection(DcMotor.Direction.REVERSE);

        RobotAuto robotAuto = new RobotAuto(this);
        RobotTop robotTop = new RobotTop(this);


        waitForStart();

        liftServo.setPosition(0.7);//0.7 back close
        sleep(700);
        robotAuto.rightShift(16);
        robotAuto.backward(28);

        robotTop.setTurnPosition(0.45);//0.45 arm out
        sleep(500);

        //lift up1
        leftLift.setPower(0.5);
        rightLift.setPower(0.5);
        sleep(1100);
        leftLift.setPower(0);
        rightLift.setPower(0);

        //close to target
        robotAuto.backward(3);

        //lift down1
        leftLift.setPower(-0.5);
        rightLift.setPower(-0.5);
        sleep(250);
        leftLift.setPower(0);
        rightLift.setPower(0);

        leftLift.setPower(-0.5);
        rightLift.setPower(-0.5);
        sleep(100);
        liftServo.setPosition(0.1);//back open
        sleep(100);
        leftLift.setPower(0);
        rightLift.setPower(0);

        robotAuto.fastForward(20);

        robotAuto.spin(90);
        robotAuto.fastForward(45);
        robotAuto.spin(180);
        liftServo.setPosition(0.1);//0.1 back open
        robotAuto.spin(180);
        sleep(600);
        robotAuto.backward(12);
        robotAuto.forward(0.8);
        liftServo.setPosition(0.7);//0.7 back close
        sleep(500);
        liftServo.setPosition(0.7);//0.7 back close

        leftLift.setPower(0.5);
        rightLift.setPower(0.5);
        sleep(400);
        leftLift.setPower(0);
        rightLift.setPower(0);
        sleep(300);

        sleep(500);
        robotAuto.forward(10);
        robotAuto.spin(270);
        robotAuto.fastForward(40);
        robotAuto.spin(0);
        robotAuto.backward(17.5);
        robotAuto.spin(1);

        robotTop.setTurnPosition(0.45);//0.45 arm out
        sleep(500);

        //lift up2
        rightLift.setPower(0.5);
        leftLift.setPower(0.5);
        sleep(1100);
        leftLift.setPower(0);
        rightLift.setPower(0);

        //close to target
        robotAuto.backward(3);

        //lift down2
        leftLift.setPower(-0.5);
        rightLift.setPower(-0.5);
        sleep(250);
        leftLift.setPower(0);
        rightLift.setPower(0);

        leftLift.setPower(-0.5);
        rightLift.setPower(-0.5);
        sleep(100);
        liftServo.setPosition(0.1);//back open
        sleep(100);
        leftLift.setPower(0);
        rightLift.setPower(0);
        sleep(100);

        robotTop.setTurnPosition(0.25);//0.25 arm in
        sleep(500);

        robotAuto.fastForward(10);
        robotAuto.spin(90);
        robotAuto.fastForward(60);
        robotAuto.rightShift(20);

    }
}