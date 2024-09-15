package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ManualOpMode extends TaskOpMode {

    private RobotTop top;

    @Override
    public void linearInit() {
        top = new RobotTop(this);
    }

    @Override
    public void linearLoop() {
        if (gamepad1.a && !gamepad1Snapshot.a) {
            top.stretchArm();
        } else if (!gamepad1.a && gamepad1Snapshot.a) {
            top.retractArm();
        }
    }
}
