//测试摄像头角度识别功能
package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.RobotVision2;

@TeleOp



public class VisionCameraAngleTest extends LinearOpMode {
    private RobotVision2 robotVision2;

    @Override
    public void runOpMode() {
        robotVision2 = new RobotVision2();

        robotVision2.initialize(hardwareMap); // 初始化摄像头

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 显示角度检测结果
            telemetry.addData("Detected Angle", robotVision2.getDetectedAngle());
            telemetry.update();

            sleep(100);
        }
    }
}
