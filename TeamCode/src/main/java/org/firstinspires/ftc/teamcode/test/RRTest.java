package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RRTest")

public class RRTest extends LinearOpMode {

    private SparkFunOTOSTest sparkFunOTOS;

    @Override
    public void runOpMode() throws InterruptedException {
        // 初始化SparkFun OTOS传感器
        sparkFunOTOS = new SparkFunOTOSTest(hardwareMap);

        // 初始化驱动对象
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // 定义机器人起始位置
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90.0));
        drive.setPoseEstimate(startPose);

        // 构建轨迹序列
        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(43.40, 49.04, Math.toRadians(90.00)))
                .splineTo(new Vector2d(47.83, -32.93), Math.toRadians(-86.91))
                .splineTo(new Vector2d(18.43, -53.47), Math.toRadians(214.94))
                .splineTo(new Vector2d(-48.23, -41.99), Math.toRadians(170.23))
                .splineTo(new Vector2d(-46.22, 3.52), Math.toRadians(87.47))
                .splineTo(new Vector2d(-35.55, 47.03), Math.toRadians(76.21))
                .splineTo(new Vector2d(11.98, 52.67), Math.toRadians(6.77))
                .build();

        // 等待开始命令
        waitForStart();

        // 开始跟踪轨迹
        if (opModeIsActive()) {
            drive.followTrajectorySequence(trajectory0);

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

