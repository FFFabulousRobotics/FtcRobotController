package org.firstinspires.ftc.teamcode.RRtest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="ForwardTest", group="Tuning")
public class forwardtest extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;

    @Override
    public void runOpMode() throws InterruptedException {
        // 初始化电机（请确保名称与机器人配置一致）
        leftFront  = hardwareMap.get(DcMotor.class, "FL");
        leftBack   = hardwareMap.get(DcMotor.class, "BL");
        rightFront = hardwareMap.get(DcMotor.class, "FR");
        rightBack  = hardwareMap.get(DcMotor.class, "BR");

        // 重置编码器
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // 设置为 RUN_WITHOUT_ENCODER 模式（不进行自动速度控制）
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 提示操作步骤
        telemetry.addData("说明", "将机器人放置在瓷砖区域，对齐后按下 Play 按钮。");
        telemetry.addData("说明", "然后手动缓慢推动车子向前，直至到达瓷砖末端。");
        telemetry.addData("说明", "记录屏幕上显示的平均编码器 tick 数，并测量实际前进的距离（英寸）。");
        telemetry.update();

        waitForStart();


        // 记录起始编码器位置
        int startLF = leftFront.getCurrentPosition();
        int startLB = leftBack.getCurrentPosition();
        int startRF = rightFront.getCurrentPosition();
        int startRB = rightBack.getCurrentPosition();

        // 循环显示编码器变化
        while (opModeIsActive()) {
            // 获取当前编码器值
            int currentLF = leftFront.getCurrentPosition();
            int currentLB = leftBack.getCurrentPosition();
            int currentRF = rightFront.getCurrentPosition();
            int currentRB = rightBack.getCurrentPosition();

            // 计算各个电机的编码器增量
            int deltaLF = currentLF - startLF;
            int deltaLB = currentLB - startLB;
            int deltaRF =  startRF -  currentRF;
            int deltaRB = currentRB - startRB;

            // 计算四个电机的平均增量
            double avgTicks = (deltaLF + deltaLB + deltaRF + deltaRB) / 4.0;

            // 将各项数据发送到 telemetry
            telemetry.addData("左前电机", deltaLF);
            telemetry.addData("左后电机", deltaLB);
            telemetry.addData("右前电机", deltaRF);
            telemetry.addData("右后电机", deltaRB);
            telemetry.addData("平均编码器 tick 数", avgTicks);
            telemetry.update();

            idle();
        }
    }
}
