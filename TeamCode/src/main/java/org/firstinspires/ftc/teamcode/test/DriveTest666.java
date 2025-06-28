package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class DriveTest666 extends LinearOpMode {
    //    /                 \
    //   /                   \
    //  FR                  BR
    //<-----front
    //  FL                  BL
    //   \                   /
    //    \                 /
    IMU imu;
    DcMotor leftFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightFrontDrive;
    DcMotor rightBackDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        imu = this.hardwareMap.get(IMU.class, "imu");
        resetIMU();
        boolean isAbs = false;
        boolean b = false;

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                resetIMU();
                isAbs = true;
            } else if (gamepad1.b) {
                isAbs = false;
            }
            if(gamepad1.x) b = false;
            if(gamepad1.y) b = true;
            if(b){
                if(gamepad1.dpad_up)driveRobot(0, -1, 0);
                else if(gamepad1.dpad_down)driveRobot(0, 1, 0);
                else if(gamepad1.dpad_left)driveRobot(-1, 0, 0);
                else if(gamepad1.dpad_right)driveRobot(1, 0, 0);
                else driveRobot(0,0,0);
            }else{
                if (isAbs) {
                    absoluteDriveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
                } else {
                    driveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
                }
            }

            telemetry.addData("isAbs", isAbs);
            telemetry.update();
        }
    }

    public void driveRobot(double axial, double lateral, double yaw) {
        // eliminate the error produced by the right stick
        yaw = (double) Math.round(yaw * 10) / 10;

        if (axial == 0 && lateral == 0 & yaw == 0) {
            brakeWheels();
        } else {
            floatWheels();
        }
        telemetry.addData("FL_rotations=",leftFrontDrive.getCurrentPosition());
        telemetry.addData("BL_rotations=",leftBackDrive.getCurrentPosition());
        telemetry.addData("FR_rotations=",rightFrontDrive.getCurrentPosition());
        telemetry.addData("BR_rotations=",rightBackDrive.getCurrentPosition());
        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        double frontLeftPower = (axial - lateral + yaw) / denominator;
        double backLeftPower = (axial + lateral - yaw) / denominator;
        double frontRightPower = (axial + lateral + yaw) / denominator;
        double backRightPower = (axial - lateral - yaw) / denominator;

        leftFrontDrive.setPower(frontLeftPower/1.821);
        leftBackDrive.setPower(backLeftPower/1.583);
        rightFrontDrive.setPower(frontRightPower/0.385);
        rightBackDrive.setPower(backRightPower/1.657);
    }

    public void absoluteDriveRobot(double axial, double lateral, double yaw) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        telemetry.addData("heading",botHeading);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = axial * Math.cos(botHeading) + lateral * Math.sin(botHeading);
        double rotY = lateral * Math.cos(botHeading) - axial * Math.sin(botHeading);

        driveRobot(rotX, rotY, yaw);
    }

    public void resetIMU() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public void brakeWheels() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void floatWheels() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
