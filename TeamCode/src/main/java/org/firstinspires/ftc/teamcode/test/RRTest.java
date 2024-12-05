package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.SparkFunOTOS;

@Autonomous
public class RRTest extends LinearOpMode {

    private SampleMecanumDrive drive;
    private SparkFunOTOS otosSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        // 初始化驱动器和传感器
        drive = new SampleMecanumDrive(hardwareMap);
        otosSensor = hardwareMap.get(SparkFunOTOS.class, "otos");
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
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(new Pose2d(-33.53, -65.35, Math.toRadians(15.81)))
                .splineTo(new Vector2d(46.62, -45.21), Math.toRadians(56.31))
                .splineTo(new Vector2d(56.29, 38.97), Math.toRadians(104.04))
                .splineTo(new Vector2d(-25.07, 57.90), Math.toRadians(186.77))
                .build();

        // 等待开始指令
        waitForStart();

        if (isStopRequested()) return;

        // 执行轨迹移动
        drive.followTrajectorySequence(trajectorySequence);

        while (opModeIsActive()) {
            double distance = otosSensor.getDistance();
            telemetry.addData("Distance", distance);
            telemetry.update();
            // 实时调整逻辑
        }

    }
}
