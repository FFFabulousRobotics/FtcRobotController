package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * 这是一个多线程测试示例程序：
 * 1. 创建两个线程，均实现了 Runnable 接口；
 * 2. 在线程中调用 synchronized 方法 incrementSharedResult() 对共享变量进行加1操作；
 * 3. 主线程循环中通过 telemetry 回显当前计算结果。
 */
@TeleOp(name = "MultiThreadTest", group = "Test")
public class ThreadTest extends LinearOpMode {

    // 共享变量，保存计算结果
    private int sharedResult = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // 创建两个线程，每个线程执行 CalculationTask 任务
        Thread thread1 = new Thread(new CalculationTask());
        Thread thread2 = new Thread(new CalculationTask());

        // 等待开始指令
        waitForStart();

        thread1.start();
        thread2.start();

        // 主循环中不断通过 telemetry 输出当前的计算结果
        while (opModeIsActive()) {
            telemetry.addData("Shared Result", getSharedResult());
            telemetry.update();
            sleep(100);
        }
    }

    /**
     * 使用 synchronized 作为方法修饰符，保证加1操作的线程安全
     */
    private synchronized void incrementSharedResult() {
        sharedResult++;
    }

    /**
     * 使用 synchronized 作为方法修饰符，保证读取共享变量时的数据一致性
     */
    private synchronized int getSharedResult() {
        return sharedResult;
    }

    /**
     * CalculationTask 任务：
     * 模拟简单计算（循环加1），每次操作后短暂延时，
     * 通过调用 synchronized 方法进行线程安全的更新。
     */
    private class CalculationTask implements Runnable {
        @Override
        public void run() {
            for (int i = 0; i < 50; i++) {
                incrementSharedResult();
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
    }
}
