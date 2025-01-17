package org.firstinspires.ftc.teamcode.test;

import com.qualcomm. robotcore.eventloop. opmode.Autonomous;
import com.qualcomm. robotcore. eventloop. opmode. LinearOpMode;
import com.qualcomm. robotcore.hardware.DcMotorSimple;

@Autonomous
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorSimple frontLeft = hardwareMap.get(DcMotorSimple.class, "FL");
        DcMotorSimple backLeft = hardwareMap.get(DcMotorSimple.class, "BL");
        DcMotorSimple frontRight = hardwareMap.get(DcMotorSimple.class, "FR");
        DcMotorSimple backRight = hardwareMap.get(DcMotorSimple.class, "BR");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        frontLeft.setPower(0.5);
        backLeft.setPower(0.5);
        frontRight.setPower(0.5);
        backRight.setPower(0.5);

        sleep(500);
    }
}