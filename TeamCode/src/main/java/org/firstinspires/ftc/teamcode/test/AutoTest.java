package org.firstinspires.ftc.teamcode.test;

import com.qualcomm. robotcore.eventloop. opmode.Autonomous;
import com.qualcomm. robotcore. eventloop. opmode. LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.comp.Todo;

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

        robotAuto.leftShift(40);
        robotAuto.spin(180);
        liftServo.setPosition(0.1);//0.1 back open
        robotAuto.backward(12);
        liftServo.setPosition(0.7);//0.7 back close
        sleep(150);

        leftLift.setPower(0.5);
        rightLift.setPower(0.5);
        sleep(200);
        leftLift.setPower(0);
        rightLift.setPower(0);

//        robotTop.setTurnPosition(0.25);//0.25 in
//        sleep(500);

        robotAuto.forward(10);
        robotAuto.leftShift(40);
        robotAuto.spin(0);
        robotAuto.fastBackward(10);

        robotTop.setTurnPosition(0.45);//0.45 arm out
        sleep(500);

        //lift up2
        rightLift.setPower(0.5);
        leftLift.setPower(0.5);
        sleep(1100);
        leftLift.setPower(0);
        rightLift.setPower(0);

        //close to target
        robotAuto.backward(5);

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

        robotAuto.fastForward(5);
        robotAuto.leftShift(60);
        robotAuto.fastForward(15);

    }
}