package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.hardware.RobotChassis;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;

@TeleOp(group = "Test")
public class DriveTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotChassis robotChassis = new RobotChassis(this);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad2.a){
                robotChassis.resetIMU();
            }
            robotChassis.absoluteDriveRobot(gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x);
            sleep(10);
        }
    }
}