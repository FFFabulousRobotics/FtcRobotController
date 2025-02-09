package org.firstinspires.ftc.teamcode.test;

import com.qualcomm. robotcore.eventloop. opmode.Autonomous;
import com.qualcomm. robotcore. eventloop. opmode. LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;

@Autonomous(group = "Test")
public class SmallAutoTest extends LinearOpMode {
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

        robotAuto.gotoPos(10,0);
        robotAuto.gotoPos(-10,0);
    }
}
