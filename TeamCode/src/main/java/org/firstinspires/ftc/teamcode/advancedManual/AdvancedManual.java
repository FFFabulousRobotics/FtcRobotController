package org.firstinspires.ftc.teamcode.advancedManual;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

@Disabled
// The ManualOp currently used is a piece of shit.
// I'll write a better one here (if the time is enough).
public class AdvancedManual extends LinearOpMode {
    @Override
    public void runOpMode(){
        TaskManager servoMgr = new TaskManager();
        Gamepad previousGamepad1 = new Gamepad();
        previousGamepad1.copy(gamepad1);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //code

            servoMgr.updateServos();
            previousGamepad1.copy(gamepad1);
            sleep(50);
        }
    }
}
