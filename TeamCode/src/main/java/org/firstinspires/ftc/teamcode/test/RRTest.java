package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

        // 定义目标位置
        Pose2d targetPosition = new Pose2d(30, 30, Math.toRadians(0)); // 示例目标位置

        // 创建轨迹
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(targetPosition.getX(), targetPosition.getY()))
                .build();

        // 等待开始指令
        waitForStart();

        if (isStopRequested()) return;

        // 执行轨迹移动
        drive.followTrajectory(trajectory);

        while (opModeIsActive()) {
            double distance = otosSensor.getDistance();
            telemetry.addData("Distance", distance);
            telemetry.update();
            // 实时调整逻辑
        }

    }
}
