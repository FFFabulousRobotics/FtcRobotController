package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

@Autonomous(name="Dual Distance Sensor Test", group="Sensor")
public class DistanceSensorTest extends LinearOpMode {

    private Rev2mDistanceSensor leftDistanceSensor;
    private Rev2mDistanceSensor rightDistanceSensor;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // 初始化传感器
        leftDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rightDistanceSensor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // 等待启动命令
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // 获取左侧距离数据
            double leftDistance = leftDistanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Left Distance (cm)", leftDistance);

            // 获取右侧距离数据
            double rightDistance = rightDistanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Right Distance (cm)", rightDistance);

            // 显示数据
            telemetry.update();
        }
    }
}
