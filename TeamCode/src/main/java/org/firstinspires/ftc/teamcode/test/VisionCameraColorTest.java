package org.firstinspires.ftc.teamcode.test;//测试摄像头颜色识别功能
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.RobotVision.RobotVisionColor;

@Disabled
@TeleOp(group = "Test")
public class VisionCameraColorTest extends LinearOpMode {
    private RobotVisionColor robotVisionColor;

    @Override
    public void runOpMode() {
        robotVisionColor = new RobotVisionColor();

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


