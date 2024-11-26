//测试摄像头颜色识别功能
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.RobotVision;

@TeleOp

public class TEST2 extends LinearOpMode {
    private RobotVision robotVision;

    @Override
    public void runOpMode() {
        robotVision = new RobotVision();

        robotVision.initialize(hardwareMap); // 初始化摄像头

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 显示颜色检测结果
            telemetry.addData("Center Color", robotVision.getDetectedColor());
            telemetry.update();

            sleep(100);
        }
    }
}


