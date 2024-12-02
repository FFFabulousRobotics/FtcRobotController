package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous (name = "RRTest")

public class RRTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // 初始化驱动对象
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // 定义机器人起始位置
        Pose2d startPose = new Pose2d(23.26, -68.58, Math.toRadians(90.00));
        drive.setPoseEstimate(startPose);

        // 构建轨迹序列
        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(26.79, 64.23), Math.toRadians(182.66))
                .splineTo(new Vector2d(16.43, 44.71), Math.toRadians(242.05))
                .splineTo(new Vector2d(45.31, 35.15), Math.toRadians(-18.32))
                .splineTo(new Vector2d(46.90, 62.44), Math.toRadians(-33.69))
                .build();

        // 等待开始命令
        waitForStart();
        // 开始跟踪轨迹
        if (opModeIsActive()) {
            drive.followTrajectorySequence(trajectory0);

            while (opModeIsActive()) {
                drive.update();
                telemetry.addData("Current Pose", drive.getPoseEstimate());
                telemetry.update();
            }
        }
    }
}

