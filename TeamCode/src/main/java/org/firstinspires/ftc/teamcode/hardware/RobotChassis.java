package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@SuppressWarnings(value = "unused")
public class RobotChassis {
    LinearOpMode opMode;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    protected DcMotor leftFrontDrive;
    protected DcMotor leftBackDrive;
    protected DcMotor rightFrontDrive;
    protected DcMotor rightBackDrive;
    protected IMU imu;

    public RobotChassis(LinearOpMode opMode) {
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        this.initMovement();
    }

    public void initMovement() {
        // init motors and servos
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        resetEncoder();

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //IMU
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        resetIMU();
    }

    public void resetIMU(){
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public void resetEncoder() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isAllBusy() {
        return leftFrontDrive.isBusy() &&
                leftBackDrive.isBusy() &&
                rightFrontDrive.isBusy() &&
                rightBackDrive.isBusy();
    }

    /**
     * Calculates the motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param axial   Forward/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param lateral Right/Left driving power (-1.0 to 1.0) +ve is forward
     * @param yaw     Right/Left turning power (-1.0 to 1.0) +ve is clockwise
     */
    public void driveRobot(double axial, double lateral, double yaw) {
        lateral = -lateral * 1.1; // Counteract imperfect strafing
        yaw = -yaw;

//        final double LEFT_REDUCTION = 0.96;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
//        double frontLeftPower = (axial + lateral + yaw) / denominator * LEFT_REDUCTION;
//        double backLeftPower = (axial - lateral + yaw) / denominator * LEFT_REDUCTION;
        double frontLeftPower = (axial + lateral + yaw) / denominator;
        double backLeftPower = (axial - lateral + yaw) / denominator;
        double frontRightPower = (axial - lateral - yaw) / denominator;
        double backRightPower = (axial + lateral - yaw) / denominator;
//        telemetry.addData("fl",frontLeftPower);
//        telemetry.addData("bl",backLeftPower);
//        telemetry.addData("fr",frontRightPower);
//        telemetry.addData("br",backRightPower);
//        telemetry.update();

        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
    }

    public void absoluteDriveRobot(double axial, double lateral, double yaw){
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = axial * Math.cos(-botHeading) - lateral * Math.sin(-botHeading);
        double rotY = axial * Math.sin(-botHeading) + lateral * Math.cos(-botHeading);

        driveRobot(rotX, rotY, yaw);
    }

    /**
     * Set the power of every motor.
     *
     * @param leftFrontPower  Forward/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightFrontPower Forward/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param leftBackPower   Forward/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightBackPower  Forward/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Stop All drive motors.
     */
    public void stopMotor() {
        setDrivePower(0, 0, 0, 0);
    }

    /**
     * Set drive wheels run mode for all drive motors.
     *
     * @param mode The desired run mode.
     */
    public void setRunMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    /**
     * Set zero power behavior for all drive motors.
     *
     * @param behavior The desired zero power behaviour.
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }

    /**
     * Set target position for all drive motors for moving forward.
     *
     * @param moveCounts The desired increment/decrement.
     */
    public void setTargetPosition(int[] moveCounts) {
        // Set Target FIRST, then turn on RUN_TO_POSITION
        leftFrontDrive.setTargetPosition(moveCounts[0]);
        rightFrontDrive.setTargetPosition(moveCounts[1]);
        leftBackDrive.setTargetPosition(moveCounts[2]);
        rightBackDrive.setTargetPosition(moveCounts[3]);
    }

    public int[] getTargetPosition() {
        // Set Target FIRST, then turn on RUN_TO_POSITION
        int lf = leftFrontDrive.getTargetPosition();
        int rf = rightFrontDrive.getTargetPosition();
        int lb = leftBackDrive.getTargetPosition();
        int rb = rightBackDrive.getTargetPosition();
        return new int[]{lf, lb, lb, rb};
    }

    public int[] getCurrentPosition() {
        // Set Target FIRST, then turn on RUN_TO_POSITION
        int lf = leftFrontDrive.getCurrentPosition();
        int rf = rightFrontDrive.getCurrentPosition();
        int lb = leftBackDrive.getCurrentPosition();
        int rb = rightBackDrive.getCurrentPosition();
        return new int[]{lf, lb, lb, rb};
    }

}
