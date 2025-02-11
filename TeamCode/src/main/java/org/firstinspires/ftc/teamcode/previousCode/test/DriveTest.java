package org.firstinspires.ftc.teamcode.previousCode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotChassis;

@TeleOp(group = "Test")
public class DriveTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotChassis robotChassis = new RobotChassis(this);

        waitForStart();

        if (isStopRequested()) return;
        boolean isAbsolute = true;
        boolean flag = true;

        while (opModeIsActive()) {
            if(gamepad1.a){
                robotChassis.resetIMU();
            }
            if(gamepad1.b && flag){
                isAbsolute = !isAbsolute;
                flag = false;
            }else{
                flag = true;
            }
            if(isAbsolute){
                robotChassis.absoluteDriveRobot(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }else {
                robotChassis.driveRobot(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
            sleep(50);
        }
    }
}