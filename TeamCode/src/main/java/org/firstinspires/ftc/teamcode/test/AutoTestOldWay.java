package org.firstinspires.ftc.teamcode.test;

import com.qualcomm. robotcore.eventloop. opmode.Autonomous;
import com.qualcomm. robotcore. eventloop. opmode. LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;

@Autonomous(group = "Test")
public class AutoTestOldWay extends LinearOpMode {
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

        robotAuto.grab();
        sleep(200);
        robotAuto.grab();
        robotAuto.gotoPos(20,-26);
//        robotAuto.rightShift(16);
//        robotAuto.backward(28);
        robotAuto.spin(0);

        robotTop.setTurnPosition(0.6);//0.6 arm out
        sleep(500);

        //lift up1
        leftLift.setPower(0.5);
        rightLift.setPower(0.5);
        sleep(1100);
        leftLift.setPower(0);
        rightLift.setPower(0);

        //close to target
        robotAuto.backward(3.5);

        //lift down1
        leftLift.setPower(-0.5);
        rightLift.setPower(-0.5);
        sleep(250);
        leftLift.setPower(0);
        rightLift.setPower(0);

        sleep(100);
        robotAuto.release();

        leftLift.setPower(-0.5);
        rightLift.setPower(-0.5);
        sleep(500);
        leftLift.setPower(0);
        rightLift.setPower(0);
        sleep(250);
        robotTop.setTurnPosition(0.3);//0.3 arm in
        sleep(300);

        robotAuto.fastForward(20);

        robotAuto.spin(90);
        robotAuto.fastForward(36);
        robotAuto.spin(90);
        robotAuto.gotoPos(-20,-44);
        robotAuto.gotoPos(-27,-44);
        robotAuto.gotoPos(-27,-5);
        robotAuto.spin(180);

        //AUTO阶段推剩余两个sample
//        robotAuto.leftShift(50);
//        robotAuto.spin(90);
//        robotAuto.forward(8);
//        robotAuto.rightShift(52);
//        robotAuto.spin(90);
//        robotAuto.leftShift(54);
//        robotAuto.spin(90);
//        robotAuto.forward(10);
//        robotAuto.rightShift(56);
//        robotAuto.spin(90);

        robotAuto.release();
        robotAuto.spin(180);
        robotAuto.backward(4.5);
        robotAuto.grab();

        //小幅度抬升使标本脱离
        leftLift.setPower(0);
        rightLift.setPower(0);
        sleep(500);
        leftLift.setPower(0.5);
        rightLift.setPower(0.5);
        sleep(400);
        leftLift.setPower(0);
        rightLift.setPower(0);

        sleep(300);
        robotAuto.gotoPos(10,-21);
        robotAuto.spin(1);
        //最古早
//        robotAuto.forward(5);
//        robotAuto.spin(270);
//        robotAuto.fastForward(40);
//        robotAuto.spin(0);
//        robotAuto.backward(17.5);
//        robotAuto.spin(1);

        robotTop.setTurnPosition(0.6);//0.6 arm out
        robotAuto.spin(1);
        sleep(500);

        //lift up2
        rightLift.setPower(0.5);
        leftLift.setPower(0.5);
        sleep(1100);
        leftLift.setPower(0);
        rightLift.setPower(0);

        //close to target
        robotAuto.backward(3.5);

        //lift down2
        leftLift.setPower(-0.5);
        rightLift.setPower(-0.5);
        sleep(250);
        leftLift.setPower(0);
        rightLift.setPower(0);

        sleep(100);
        robotAuto.release();

        leftLift.setPower(-0.5);
        rightLift.setPower(-0.5);
        sleep(500);
        leftLift.setPower(0);
        rightLift.setPower(0);
        sleep(250);

        robotTop.setTurnPosition(0.3);//0.3 arm in
        sleep(300);

        robotAuto.fastForward(20);

        robotAuto.gotoPos(-27,-5);
        robotAuto.spin(180);

        robotAuto.release();
        robotAuto.spin(180);
        robotAuto.backward(7.3);
        robotAuto.grab();

        //小幅度抬升使标本脱离
        leftLift.setPower(0);
        rightLift.setPower(0);
        sleep(600);
        leftLift.setPower(0.5);
        rightLift.setPower(0.5);
        sleep(400);
        leftLift.setPower(0);
        rightLift.setPower(0);

        sleep(500);
        robotAuto.gotoPos(13,-23);
        robotAuto.spin(1);
        sleep(100);

        robotTop.setTurnPosition(0.6);//0.6 arm out
        robotAuto.spin(1);
        sleep(100);

        //lift up3
        rightLift.setPower(0.5);
        leftLift.setPower(0.5);
        sleep(1100);
        leftLift.setPower(0);
        rightLift.setPower(0);

        //close to target
        robotAuto.backward(2);

        //lift down3
        leftLift.setPower(-0.5);
        rightLift.setPower(-0.5);
        sleep(250);
        leftLift.setPower(0);
        rightLift.setPower(0);

        sleep(100);
        robotAuto.release();

        leftLift.setPower(-0.5);
        rightLift.setPower(-0.5);
        sleep(250);
        leftLift.setPower(0);
        rightLift.setPower(0);
        sleep(100);

        robotTop.setTurnPosition(0.3);//0.3 arm in
        sleep(300);

        robotAuto.gotoPos(-45,0);

//        robotAuto.spin(90);
//        robotAuto.fastForward(45);
//        robotAuto.spin(180);
//        liftServo.setPosition(0.1);//0.1 back open
//        robotAuto.spin(180);
//        sleep(600);
//        robotAuto.backward(12);
//        robotAuto.forward(0.8);
//        liftServo.setPosition(0.7);//0.7 back close
//        sleep(500);
//        liftServo.setPosition(0.7);//0.7 back close
//
//        robotAuto.fastForward(10);
//        robotAuto.spin(90);
//        robotAuto.fastForward(60);
//        robotAuto.rightShift(20);

    }
}