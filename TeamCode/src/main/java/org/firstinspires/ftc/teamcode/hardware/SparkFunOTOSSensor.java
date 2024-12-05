package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SparkFunOTOSSensor {
    private SparkFunOTOS otosDevice;
    private double x;
    private double y;
    private double heading;

    public SparkFunOTOSSensor(HardwareMap hardwareMap) {
        // 假设 OTOS 传感器连接到 I2C 端口
        otosDevice = hardwareMap.get(SparkFunOTOS.class, "otos");
        // 初始化传感器（具体初始化代码取决于 OTOS 库的实现）
        this.x = 0.0;
        this.y = 0.0;
        this.heading = 0.0;
    }

    public double getX() {
        // 返回 X 坐标
        return x; // 这里应从传感器获取实际数据
    }

    public double getY() {
        // 返回 Y 坐标
        return y; // 这里应从传感器获取实际数据
    }

    public double getHeading() {
        // 返回方向（角度）
        return heading; // 这里应从传感器获取实际数据
    }

    public void updateData(double newX, double newY, double newHeading) {
        // 更新传感器数据
        this.x = newX;
        this.y = newY;
        this.heading = newHeading;
    }

    public double getDistance() {

        return 0;
    }
}

