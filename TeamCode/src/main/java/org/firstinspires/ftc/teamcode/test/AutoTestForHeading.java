package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.hardware.RobotChassis;

@TeleOp(group = "Test")
public class AutoTestForHeading extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotChassis robotChassis = new RobotChassis(this);
        RobotAuto robotAuto = new RobotAuto(this);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D pos = robotAuto.getPosition();
            // Log the position to the telemetry
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);
            telemetry.update();
            if(gamepad1.a){
                robotAuto.gotoPos(0,0);
                telemetry.update();
            }
            if(gamepad1.b){
                robotAuto.gotoPosWithHeading(0,0, 0);
                telemetry.update();
            }
            robotChassis.driveRobot(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            sleep(50);
        }
    }
}
