package org.firstinspires.ftc.teamcode.test;//测试摄像头角度识别功能

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.RobotVisionAngle;

@TeleOp



public class VisionCameraAngleTest extends LinearOpMode {
    private RobotVisionAngle robotVisionAngle;

    @Override
    public void runOpMode() {
        robotVisionAngle = new RobotVisionAngle();

        robotVisionAngle.initialize(hardwareMap); // 初始化摄像头

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 显示角度检测结果
            telemetry.addData("Detected Angle", robotVisionAngle.getDetectedAngle());
            telemetry.update();

            sleep(100);
        }
    }
}
