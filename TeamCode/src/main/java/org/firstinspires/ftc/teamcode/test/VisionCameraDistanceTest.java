package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotVision.RobotCameraDistance;

@Disabled
@TeleOp(name = "VisionCameraDistanceTest", group = "Test")
public class VisionCameraDistanceTest extends LinearOpMode {
    private RobotCameraDistance robotCameraDistance;

    @Override
    public void runOpMode() {
        robotCameraDistance = new RobotCameraDistance();

        robotCameraDistance.initialize(hardwareMap, "blue");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double[] polarCoordinates = robotCameraDistance.getPolarCoordinates();

            if (gamepad1.a) {
                robotCameraDistance.updateCurrentRectExcludingCurrent();
            }

            telemetry.addData("Distance from Center", polarCoordinates[0]);
            telemetry.addData("Angle from Center", polarCoordinates[1]);
            telemetry.update();

            sleep(50);
        }
    }
}
