package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotAutoEasy {
    LinearOpMode opMode;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;

    final static double ForwardRatio = 600;
    final static double SpinRatio = 50;
    final static double ShiftRatio = 50;

    public RobotAutoEasy(LinearOpMode opMode) {
        this.opMode = opMode;
        this.init();
    }

    private void init() {
        frontLeft = opMode.hardwareMap.get(DcMotor.class, "FL");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "BL");
        frontRight = opMode.hardwareMap.get(DcMotor.class, "FR");
        backRight = opMode.hardwareMap.get(DcMotor.class, "BR");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public RobotAutoEasy forward(double distance) {
        double power = 0.7;
        long sleepTime = (long) (distance * ForwardRatio); // Example conversion factor

        setDrivePower(power, power, power, power);
        opMode.sleep(sleepTime);
        stopMotors();
        return this;
    }

    public RobotAutoEasy spin(double degree) {
        double power = 0.7;
        long sleepTime = (long) (degree * SpinRatio); // Example conversion factor

        setDrivePower(power, power, -power, -power);
        opMode.sleep(sleepTime);
        stopMotors();
        return this;
    }

    public RobotAutoEasy shift(double distance) {
        double power = 0.7;
        long sleepTime = (long) (distance * ShiftRatio); // Example conversion factor
        setDrivePower(power, -power, power, -power);
        opMode.sleep(sleepTime);
        stopMotors();
        return this;
    }

    private void setDrivePower(double leftFront, double leftBack, double rightFront, double rightBack) {
        frontLeft.setPower(leftFront);
        backLeft.setPower(leftBack);
        frontRight.setPower(rightFront);
        backRight.setPower(rightBack);
    }

    private void stopMotors() {
        setDrivePower(0, 0, 0, 0);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    private void setRunMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        backLeft.setMode(mode);
        frontRight.setMode(mode);
        backRight.setMode(mode);
    }
}

