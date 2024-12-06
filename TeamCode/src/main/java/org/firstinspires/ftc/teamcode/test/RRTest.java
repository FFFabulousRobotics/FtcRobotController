package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.SparkFunOTOSSensor;

@Autonomous
public class RRTest extends LinearOpMode {

    private SampleMecanumDrive drive;
    private SparkFunOTOSSensor otosSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        // 初始化驱动器和传感器
        drive = new SampleMecanumDrive(hardwareMap);
        // 创建轨迹序列
//        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 90))
//                .forward(100)
//                .turn(Math.toRadians(90))
//                .forward(100)
//                .addDisplacementMarker(() -> {
//                    // 在这里添加要执行的操作，如发射器射击或手臂放下
//                    // bot.shooter.shoot();
//                    // bot.wobbleArm.lower();
//                })
//                .turn(Math.toRadians(90))
//                .splineTo(new Vector2d(50, 15), Math.toRadians(0))
//                .turn(Math.toRadians(90))
//                .build();
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(new Pose2d(36.35, -62.84, Math.toRadians(47.22)))
                .splineTo(new Vector2d(60.65, -35.75), Math.toRadians(90.00))
                .splineTo(new Vector2d(61.24, 37.15), Math.toRadians(90.00))
                .splineTo(new Vector2d(35.95, 60.05), Math.toRadians(180.00))
                .build();
        // 等待开始指令
        waitForStart();

        if (isStopRequested()) return;

        // 执行轨迹移动
        drive.followTrajectorySequence(trajectorySequence);


    }
}
