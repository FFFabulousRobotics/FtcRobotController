package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotChassis;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;

@TeleOp
public class ManualOpMode extends LinearOpMode {
    RobotChassis robotChassis;
    RobotTop robotTop;

    @Override
    public void runOpMode() {
        robotChassis = new RobotChassis(this);
        robotTop = new RobotTop(this);

        int liftPosition;
        LiftState liftState = LiftState.BOTTOM;
        waitForStart();
        while (opModeIsActive()){
            robotChassis.driveRobot(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            liftPosition = robotTop.getLiftPosition();
            if(liftState == LiftState.BOTTOM){
                if(gamepad1.y && liftPosition <= 1100){
                    robotTop.setLeftPower(0.7);
                }else if (liftPosition >= 1250) {
                    robotTop.setLeftPower(0);
                    liftState = LiftState.TOP;
                } else if (liftPosition >= 1100) {
                    robotTop.setLeftPower(0.3);
                }
            } else if (liftState == LiftState.TOP) {
                if(gamepad1.y && liftPosition >= 200){
                    robotTop.setLeftPower(-0.6);
                } else if (liftPosition <= 50) {
                    robotTop.setLeftPower(0);
                    liftState = LiftState.BOTTOM;
                }else if (liftPosition <= 200) {
                    robotTop.setLeftPower(-0.2);
                }
            }
            telemetry.addData("pos", liftPosition);
            telemetry.addData("state", liftState);
            telemetry.update();
            sleep(10);
        }
    }
}
enum LiftState{
    BOTTOM,TOP
}