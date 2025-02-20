package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


@SuppressWarnings(value = "unused")
@Config
public class RobotChassis {
    LinearOpMode opMode;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    // Updated motor declarations to DcMotorEx
    protected DcMotorEx leftFrontDrive;
    protected DcMotorEx leftBackDrive;
    protected DcMotorEx rightFrontDrive;
    protected DcMotorEx rightBackDrive;
    protected IMU imu;

    // PIDF coefficients (adjust these values as needed)
    public static final double NEW_P = 0.6;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.0;
    public static final double NEW_F = 0.0 ;

    public RobotChassis(LinearOpMode opMode) {
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        this.initMovement();
    }

    public void initMovement() {
        // Initialize motors as DcMotorEx
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "FL");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "BL");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "FR");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "BR");

        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders
        resetEncoder();

        // Set zero power behavior
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        leftFrontDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        leftBackDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightFrontDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightBackDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Ensure motors run using encoders
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize IMU
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        resetIMU();
    }

    public void resetIMU() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public void resetEncoder() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
     * robot motions: Drive (Axial motion), Lateral motion, and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param axial   Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param lateral Right/Left strafing power (-1.0 to 1.0) +ve is right
     * @param yaw     Right/Left turning power (-1.0 to 1.0) +ve is clockwise
     */
    public void driveRobot(double axial, double lateral, double yaw) {
        lateral = -lateral; // Counteract imperfect strafing
        yaw = -yaw;

        // Normalize the values so no value exceeds 1.0
        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1.0);

        double frontLeftPower = (axial + lateral + yaw) / denominator;
        double backLeftPower = (axial - lateral + yaw) / denominator;
        double frontRightPower = (axial - lateral - yaw) / denominator;
        double backRightPower = (axial + lateral - yaw) / denominator;

        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
    }

    public void absoluteDriveRobot(double axial, double lateral, double yaw) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = axial * Math.cos(-botHeading) - lateral * Math.sin(-botHeading);
        double rotY = axial * Math.sin(-botHeading) + lateral * Math.cos(-botHeading);

        driveRobot(rotX, rotY, yaw);
    }

    /**
     * Set the power of each motor individually.
     *
     * @param leftFrontPower  Power for left front motor (-1.0 to 1.0)
     * @param rightFrontPower Power for right front motor (-1.0 to 1.0)
     * @param leftBackPower   Power for left back motor (-1.0 to 1.0)
     * @param rightBackPower  Power for right back motor (-1.0 to 1.0)
     */
    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Stop all drive motors.
     */
    public void stopMotor() {
        setDrivePower(0, 0, 0, 0);
    }

    /**
     * Set the run mode for all drive motors.
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
     * @param behavior The desired zero power behavior.
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }

    /**
     * Set target position for all drive motors.
     *
     * @param moveCounts Array containing target positions for each motor.
     */
    public void setTargetPosition(int[] moveCounts) {
        leftFrontDrive.setTargetPosition(moveCounts[0]);
        rightFrontDrive.setTargetPosition(moveCounts[1]);
        leftBackDrive.setTargetPosition(moveCounts[2]);
        rightBackDrive.setTargetPosition(moveCounts[3]);
    }

    public int[] getTargetPosition() {
        int lf = leftFrontDrive.getTargetPosition();
        int rf = rightFrontDrive.getTargetPosition();
        int lb = leftBackDrive.getTargetPosition();
        int rb = rightBackDrive.getTargetPosition();
        return new int[]{lf, rf, lb, rb};
    }

    public int[] getCurrentPosition() {
        int lf = leftFrontDrive.getCurrentPosition();
        int rf = rightFrontDrive.getCurrentPosition();
        int lb = leftBackDrive.getCurrentPosition();
        int rb = rightBackDrive.getCurrentPosition();
        return new int[]{lf, rf, lb, rb};
    }
}
