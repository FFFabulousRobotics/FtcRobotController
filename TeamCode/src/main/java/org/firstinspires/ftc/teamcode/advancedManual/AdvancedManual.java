package org.firstinspires.ftc.teamcode.advancedManual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

// The ManualOp currently used is a piece of shit.
// I'll write a better one here (if the time is enough).
public class AdvancedManual extends LinearOpMode {
    @Override
    public void runOpMode(){
        ServoManager servoMgr = new ServoManager();
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
