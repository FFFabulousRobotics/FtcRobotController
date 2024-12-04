package org.firstinspires.ftc.teamcode.test;//测试摄像头距离和角度识别功能

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.RobotVision.RobotCameraDistance;

@TeleOp(name = "VisionCameraDistanceTest", group = "Test")
public class VisionCameraDistanceTest extends LinearOpMode {
    private RobotCameraDistance robotCameraDistance;

    @Override
    public void runOpMode() {
        robotCameraDistance = new RobotCameraDistance();

        robotCameraDistance.initialize(hardwareMap, "blue"); // 初始化摄像头，并设置颜色参数为红色

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 获取极坐标
            double[] polarCoordinates = robotCameraDistance.getPolarCoordinates();

            // 显示距离和角度检测结果
            telemetry.addData("Distance from Center", polarCoordinates[0]);
            telemetry.addData("Angle from Center", polarCoordinates[1]);
            telemetry.update();

            sleep(50);
        }
    }
}
