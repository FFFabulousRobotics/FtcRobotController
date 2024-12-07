package org.firstinspires.ftc.teamcode.advancedManual.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.advancedManual.TaskManager;
@Disabled
// The ManualOp currently used is a piece of shit.
// I'll write a better one here (if the time is enough).
public class TimedServoTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        Servo servo = hardwareMap.get(Servo.class, "servo");
        TaskManager servoMgr = new TaskManager();
        Gamepad previousGamepad1 = new Gamepad();
        previousGamepad1.copy(gamepad1);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad1.a && !previousGamepad1.a){
                servoMgr.setTimedServoPosition(servo, 1,2000);
            }
            if(gamepad1.b && !previousGamepad1.b){
                servoMgr.setTimedServoPosition(servo, 0,2000);
            }

            servoMgr.updateServos();
            previousGamepad1.copy(gamepad1);
            sleep(50);
        }
    }
}
