package org.firstinspires.ftc.teamcode.roadrunner.RRtest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name="Lateral Push Test", group="Tuning")
public class LateralPushTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // 创建 MecanumDrive 实例
        // 注意：此处传入的初始位姿设为 null，因为本 op mode 仅用于测量编码器 tick 增量
        MecanumDrive drive = new MecanumDrive(hardwareMap, null);

        // 重置所有电机的编码器
        drive.leftFront.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftBack.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightFront.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightBack.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // 设置为 RUN_WITHOUT_ENCODER 模式（只读取编码器数值，不进行自动调速）
        drive.leftFront.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.leftBack.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightFront.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightBack.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("准备就绪！");
        telemetry.addLine("请按 Play 按钮，然后手动将机器人向左推");
        telemetry.addLine("同时记录屏幕上显示的 lateral tick 数和实际横向移动的距离（英寸）");
        telemetry.update();

        waitForStart();

        // 记录初始编码器值
        int initialFL = drive.leftFront.getCurrentPosition();
        int initialBL = drive.leftBack.getCurrentPosition();
        int initialFR = drive.rightFront.getCurrentPosition();
        int initialBR = drive.rightBack.getCurrentPosition();

        while (opModeIsActive()) {
            int currentFL = drive.leftFront.getCurrentPosition();
            int currentBL = drive.leftBack.getCurrentPosition();
            int currentFR = drive.rightFront.getCurrentPosition();
            int currentBR = drive.rightBack.getCurrentPosition();

            int deltaFL = currentFL - initialFL;
            int deltaBL = currentBL - initialBL;
            int deltaFR = initialFR - currentFR ;
            int deltaBR = currentBR - initialBR;

            // 对于横向推测试，
            // 当向左推时，通常前左与后右电机的编码器值会增加，
            // 而前右与后左的编码器值会减少。
            // 因此，我们可以使用公式：
            // lateralTicks = (deltaFL + deltaBR - deltaFR - deltaBL) / 4.0
            // 得到一个正数，表示横向移动所产生的编码器 tick 数。
            double lateralTicks = ( (deltaFL + deltaBR) - (deltaFR + deltaBL) ) / 4.0;

            telemetry.addData("左前编码器增量", deltaFL);
            telemetry.addData("左后编码器增量", deltaBL);
            telemetry.addData("右前编码器增量", deltaFR);
            telemetry.addData("右后编码器增量", deltaBR);
            telemetry.addData("lateral tick 数", lateralTicks);
            telemetry.addLine("请记录 lateral tick 数，并测量横向移动的距离（英寸）");
            telemetry.update();
        }
    }
}

