//底盘上部代码
package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@SuppressWarnings(value = "unused")
public class RobotTop {
    OpMode opMode;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    protected DcMotor leftLiftMotor;
    protected DcMotor rightLiftMotor;
    protected DcMotor armStretchMotor;
    protected Servo armLeftTurnServo;
    protected Servo armRightTurnServo;
    protected Servo armLeftSpinServo;
    protected Servo armRightSpinServo;
    protected Servo armGrabServo;
    protected Servo liftServo;
    protected Servo topServo;
    protected Servo containerServo;

    protected int targetLiftPosition;

    public RobotTop(LinearOpMode opMode) {
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        this.init();
    }

    public void init() {
        leftLiftMotor = hardwareMap.get(DcMotor.class, "leftLift");
        rightLiftMotor = hardwareMap.get(DcMotor.class, "rightLift");
        armStretchMotor = hardwareMap.get(DcMotor.class, "armStretch");
        armLeftTurnServo = hardwareMap.get(Servo.class, "armTurnL");
        armRightTurnServo = hardwareMap.get(Servo.class, "armTurnR");
        armLeftSpinServo = hardwareMap.get(Servo.class, "armSpinL");
        armRightSpinServo = hardwareMap.get(Servo.class, "armSpinR");
        armGrabServo = hardwareMap.get(Servo.class, "armGrab");
        liftServo = hardwareMap.get(Servo.class, "liftServo");
        topServo = hardwareMap.get(Servo.class, "liftTop");
        containerServo = hardwareMap.get(Servo.class, "containerServo");

        leftLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightLiftMotor.setDirection(DcMotor.Direction.REVERSE);

        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armStretchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armStretchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armStretchMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setLiftPower(double power) {
        leftLiftMotor.setPower(power);
        rightLiftMotor.setPower(power);
    }

    public int getLiftPosition() {
        return leftLiftMotor.getCurrentPosition();
    }

    public void setArmStretchPosition(int position) {
        //armStretchMotor.setTargetPosition(position);
    }

    public void setStretchPower(double p) {
        armStretchMotor.setPower(p);
    }

    public int getArmStretchPosition() {
        return armStretchMotor.getCurrentPosition();
    }

    public void setArmLeftTurnPosition(double position) {
        armLeftTurnServo.setPosition(position);
    }

    public double getArmLeftTurnPosition() {
        return armLeftTurnServo.getPosition();
    }

    public void setArmRightTurnPosition(double position) {
        armRightTurnServo.setPosition(position);
    }

    public double getArmRightTurnPosition() {
        return armRightTurnServo.getPosition();
    }

    public double getTurnPosition() {
        return getArmLeftTurnPosition();
    }

    public void setTurnPosition(double position) {
        setArmLeftTurnPosition(position);
        setArmRightTurnPosition(1 - position);
    }

    public void setArmLeftSpinPosition(double position) {
        armLeftSpinServo.setPosition(position);
    }

    public double getArmLeftSpinPosition() {
        return armLeftSpinServo.getPosition();
    }

    public void setArmRightSpinPosition(double position) {
        armRightSpinServo.setPosition(position);
    }

    public double getArmRightSpinPosition() {
        return armRightSpinServo.getPosition();
    }

    public void setArmGrabPosition(double position) {
        armGrabServo.setPosition(position);
    }

    public double getArmGrabPosition() {
        return armGrabServo.getPosition();
    }

    public void setLiftServoPosition(double position) {
        liftServo.setPosition(position);
    }

    public double getLiftServoPosition() {
        return liftServo.getPosition();
    }

    public void setTopServoPosition(double position) {
        topServo.setPosition(position);
    }

    public double getTopServoPosition() {
        return topServo.getPosition();
    }

    public void setContainerServoPosition(double position) {
        containerServo.setPosition(position);
    }

    public double getContainerServoPosition() {
        return containerServo.getPosition();
    }

    public void setTargetLiftPosition(int position) {
        this.targetLiftPosition = position;
    }

    //PID
    double currentPos = 0, previousPos = 0, offset = 0;
    double targetPos = 0;
    double ticks_per_rev = 530;
    double delta = 0;
    double p = 0, i = 0, d = 0;
    final double Kp = 0.0008, Ki = 0.00001, Kd = -0.004;
    final double power_max = 0.5;
    double power = 0;

    public void updateLiftPID() {
        previousPos = currentPos;
        currentPos = getLiftPosition();
        delta = targetPos - currentPos - offset;
        if (delta > ticks_per_rev * 0.6)
            power = 1;
        else if (delta < -ticks_per_rev * 0.6)
            power = -1;
        else {
            p = Kp * delta;
            i += Ki * delta;
            d = Kd * (currentPos - previousPos);
            power = p + i + d;
            if (power > power_max)
                power = power_max;
            else if (power < -power_max)
                power = -power_max;
        }
        setLiftPower(power);
        telemetry.addData("power", power);
        telemetry.addData("p", p);
        telemetry.addData("i", i);
        telemetry.addData("d", d);
        telemetry.addData("current", currentPos);
        telemetry.addData("target", targetPos);
    }

    public void setLiftTargetPos(int pos) {
        this.targetPos = Math.min(1260, pos);
        this.targetPos = Math.max(40, targetPos);
    }

}
