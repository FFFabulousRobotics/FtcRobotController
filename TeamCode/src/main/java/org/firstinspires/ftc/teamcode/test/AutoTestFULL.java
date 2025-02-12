package org.firstinspires.ftc.teamcode.test;

import com.qualcomm. robotcore.eventloop. opmode.Autonomous;
import com.qualcomm. robotcore. eventloop. opmode. LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;

@Autonomous(group = "Test")
public class AutoTestFULL extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "FL");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "BL");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "FR");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "BR");
        DcMotor leftLift = hardwareMap.get(DcMotor.class, "leftLift");
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

        //开局
        robotAuto.grab();
        robotAuto.gotoPos(10,-20);//第一次到杆前
        robotAuto.spin(0);//定位校准

        robotTop.setTurnPosition(0.7);//0.7 arm out 打开前手
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

        //高挂回落
        leftLift.setPower(-0.5);
        rightLift.setPower(-0.5);
        sleep(500);
        leftLift.setPower(0);
        rightLift.setPower(0);
        sleep(250);
        robotTop.setTurnPosition(0.4);//0.4 arm in 回前手
        sleep(300);

        robotAuto.gotoPosWithHeading(-17,-11,90);//到位置1，准备北上
        robotAuto.gotoPosWithHeading(-17,-30,90 );//到位置2
        robotAuto.gotoPos(-22,-30);//到sample1上面
        robotAuto.gotoPosWithHeading(-22,-2.5,90);//推sample1到人玩区
        // 如果要推更多的sample，在这里写

        robotAuto.spin(180);
        robotAuto.release();//保持后手张开
        robotAuto.gotoPos(-22,2);//放入标本
        sleep(500);
        robotAuto.grab();//夹住

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
        robotAuto.gotoPosWithHeading(13,-19,0);//第二次到杆前

        //最古早
//        robotAuto.forward(5);
//        robotAuto.spin(270);
//        robotAuto.fastForward(40);
//        robotAuto.spin(0);
//        robotAuto.backward(17.5);
//        robotAuto.spin(1);

        robotTop.setTurnPosition(0.7);//0.7 arm out 前手出
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

        //高挂回落
        leftLift.setPower(-0.5);
        rightLift.setPower(-0.5);
        sleep(500);
        leftLift.setPower(0);
        rightLift.setPower(0);
        sleep(250);

        robotTop.setTurnPosition(0.4);//0.4 arm in 前手回
        sleep(300);

        robotAuto.gotoPosWithHeading(-22,1.5,180);//到标本前并且面对

        robotAuto.release();
        robotAuto.gotoPos(-20,4.5,180);//使标本进入
        sleep(500);
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
        robotAuto.gotoPosWithHeading(7,-18,0);//第三次到杆前
        sleep(100);

        robotTop.setTurnPosition(0.7);//0.7 arm out 前手出
        robotAuto.spin(0);//微调朝向
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

        ////高挂回落
        leftLift.setPower(-0.5);
        rightLift.setPower(-0.5);
        sleep(250);
        leftLift.setPower(0);
        rightLift.setPower(0);
        sleep(100);

        robotTop.setTurnPosition(0.4);//0.4 arm in 前手回
        sleep(300);

        robotAuto.gotoPosWithHeading(-40,2,0);//停靠

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