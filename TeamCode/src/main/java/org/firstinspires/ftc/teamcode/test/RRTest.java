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
                .splineTo(new Vector2d(24.67, -33.13), Math.toRadians(87.72))
                .splineTo(new Vector2d(51.86, -26.28), Math.toRadians(14.14))
                .splineTo(new Vector2d(57.10, -48.64), Math.toRadians(-76.82))
                .build();
        // 等待开始命令
        waitForStart();
        // 开始跟踪轨迹
        if (opModeIsActive()) {
            drive.followTrajectorySequence(trajectory0);
        }
    }
}

