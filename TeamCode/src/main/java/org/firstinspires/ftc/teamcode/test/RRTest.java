package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.test.SparkFunOTOS;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "SparkFunRoadRunnerAuto")
public class RRTest extends LinearOpMode {

    private SparkFunOTOS sparkFunOTOS;

    @Override
    public void runOpMode() throws InterruptedException {
        // 初始化SparkFun OTOS传感器
        sparkFunOTOS = new SparkFunOTOS(hardwareMap);

        // 初始化驱动对象
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // 定义机器人起始位置
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90.0));
        drive.setPoseEstimate(startPose);

        // 构建轨迹序列
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(30, 30), Math.toRadians(45.0))
                .splineTo(new Vector2d(60, 0), Math.toRadians(0.0))
                .build();

        // 等待开始命令
        waitForStart();

        // 开始跟踪轨迹
        if (opModeIsActive()) {
            drive.followTrajectorySequence(trajectory);

            while (opModeIsActive()) {
                // 更新位置信息
                drive.update();

                // 输出当前位置信息到Telemetry
                telemetry.addData("Current Pose", drive.getPoseEstimate());
                telemetry.addData("SparkFun OTOS X", sparkFunOTOS.getX());
                telemetry.addData("SparkFun OTOS Y", sparkFunOTOS.getY());
                telemetry.addData("SparkFun OTOS Heading", sparkFunOTOS.getHeading());
                telemetry.update();
            }
        }
    }
}

