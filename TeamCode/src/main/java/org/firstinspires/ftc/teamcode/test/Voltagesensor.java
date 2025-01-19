package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Autonomous
public class Voltagesensor extends LinearOpMode {

    private VoltageSensor voltageSensor;

    @Override
    public void runOpMode() {
        // 初始化硬件
        DcMotor FLmotor = hardwareMap.get(DcMotor.class, "FL");
        DcMotor FRmotor = hardwareMap.get(DcMotor.class, "FR");
        DcMotor BLmotor = hardwareMap.get(DcMotor.class, "BL");
        DcMotor BRmotor = hardwareMap.get(DcMotor.class, "BR");
        FRmotor.setDirection(DcMotor.Direction.REVERSE);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // 等待用户按下 "Start" 按钮
        waitForStart();

        // 主控制逻辑
        while (opModeIsActive()) {
            double targetVoltage = 14.0;  // 理想电压
            double currentVoltage = voltageSensor.getVoltage();
            double compensationFactor = targetVoltage / currentVoltage;

            double rawPower = 0.5;  // 原始设定功率
            double adjustedPower = rawPower * compensationFactor;

            // 确保功率范围在 [-1, 1]
            adjustedPower = Math.max(-1, Math.min(1, adjustedPower));

            FLmotor.setPower(adjustedPower);
            FRmotor.setPower(adjustedPower);
            BLmotor.setPower(adjustedPower);
            BRmotor.setPower(adjustedPower);

            // 输出回显信息

            telemetry.addData("Battery Voltage1", currentVoltage);
            telemetry.addData("Adjusted Motor Power", adjustedPower);
            telemetry.update();  // 更新 Telemetry
        }
    }
}
