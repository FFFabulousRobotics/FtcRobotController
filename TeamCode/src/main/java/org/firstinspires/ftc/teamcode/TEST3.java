//测试摄像头角度识别功能1
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotVision;
import org.firstinspires.ftc.teamcode.hardware.RobotVision2;

@TeleOp



public class TEST3 extends LinearOpMode {
    private RobotVision2 robotVision2;
    private RobotVision robotVision;

    @Override
    public void runOpMode() {
        robotVision2 = new RobotVision2();

        robotVision2.initialize(hardwareMap); // 初始化摄像头

        robotVision = new RobotVision();

        robotVision.initialize(hardwareMap); // 初始化摄像头

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 显示角度检测结果
            telemetry.addData("Center Color", robotVision.getDetectedColor());
            telemetry.addData("Detected Angle", robotVision2.getDetectedAngle());
            telemetry.update();

            sleep(100);
        }
    }
}
