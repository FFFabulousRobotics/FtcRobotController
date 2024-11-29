package org.firstinspires.ftc.teamcode.test;//测试摄像头颜色识别功能
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.RobotVision;

@TeleOp
public class VisionCameraColorTest extends LinearOpMode {
    private RobotVision robotVisionColor;

    @Override
    public void runOpMode() {
        robotVisionColor = new RobotVision();

        robotVisionColor.initialize(hardwareMap); // 初始化摄像头

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 显示颜色检测结果
            telemetry.addData("Center Color", robotVisionColor.getDetectedColor());
            telemetry.update();

            sleep(100);
        }
    }
}


