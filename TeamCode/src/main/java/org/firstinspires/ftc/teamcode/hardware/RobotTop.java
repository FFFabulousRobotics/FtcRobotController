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

    public RobotTop(LinearOpMode opMode) {
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        this.init();
    }

    public void init() {
        leftLiftMotor = hardwareMap.get(DcMotor.class, "leftLift");
        rightLiftMotor = hardwareMap.get(DcMotor.class, "rightLift");
        //armStretchServo = hardwareMap.get(Servo.class, "armStretch");

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

    public void setArmStretchPosition(double position){
        armStretchServo.setPosition(position);
    }

    public double getArmStretchPosition() {
        return armStretchServo.getPosition();
    }
}
