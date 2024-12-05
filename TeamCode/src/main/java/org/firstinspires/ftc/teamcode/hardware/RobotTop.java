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
    private DcMotor leftLiftMotor;
    private DcMotor rightLiftMotor;
    private Servo armStretchServo;
    private Servo armTurnServo;
    private Servo armSpinXServo;
    private Servo armSpinYServo;
    private Servo armGrabServo;
    private Servo liftServo;
    private Servo topServo;
    private Servo containerServo;

    public RobotTop(LinearOpMode opMode) {
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        this.init();
    }

    public void init() {
        leftLiftMotor = hardwareMap.get(DcMotor.class, "leftLift");
        rightLiftMotor = hardwareMap.get(DcMotor.class, "rightLift");
        armStretchServo = hardwareMap.get(Servo.class, "armStretch");
        armTurnServo = hardwareMap.get(Servo.class, "armTurn");
        armSpinXServo = hardwareMap.get(Servo.class, "armSpinX");
        armSpinYServo = hardwareMap.get(Servo.class, "armSpinY");
        armGrabServo = hardwareMap.get(Servo.class, "armGrab");
        liftServo = hardwareMap.get(Servo.class, "liftServo");
        topServo = hardwareMap.get(Servo.class, "liftTop");
        containerServo = hardwareMap.get(Servo.class, "containerServo");

        leftLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightLiftMotor.setDirection(DcMotor.Direction.REVERSE);

        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setLeftPower(double power) {
        leftLiftMotor.setPower(power);
        rightLiftMotor.setPower(power);
    }

    public int getLiftPosition() {
        return leftLiftMotor.getCurrentPosition();
    }

    public void setArmStretchPosition(double position) {
        armStretchServo.setPosition(position);
    }

    public double getArmStretchPosition() {
        return armStretchServo.getPosition();
    }

    public void setArmTurnPosition(double position) {
        armTurnServo.setPosition(position);
    }

    public double getArmTurnPosition() {
        return armTurnServo.getPosition();
    }

    public void setArmSpinXPosition(double position) {
        armSpinXServo.setPosition(position);
    }

    public double getArmSpinXPosition() {
        return armSpinXServo.getPosition();
    }

    public void setArmSpinYPosition(double position) {
        armSpinYServo.setPosition(position);
    }

    public double getArmSpinYPosition() {
        return armSpinYServo.getPosition();
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
}
