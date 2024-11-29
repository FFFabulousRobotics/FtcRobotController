package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

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
        Gamepad previousGamepad1 = new Gamepad();
        previousGamepad1.copy(gamepad1);

        int liftPosition;
        double armSpinXPos = 0.5;
        double armSpinYPos = 0.07;
        boolean armGrabbing = false;
        LiftState liftState = LiftState.BOTTOM;
        ArmState armState = ArmState.IDLE;

        waitForStart();
        while (opModeIsActive()){
            robotChassis.driveRobot(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            //robotLift
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

            //robotArm
            if(gamepad1.x && !previousGamepad1.x){
                if(armState == ArmState.IDLE){
                    robotTop.setArmStretchPosition(0.3);
                }else if(armState == ArmState.STRETCHED){
                    robotTop.setArmStretchPosition(0);
                }
            }
            if(gamepad1.a && !previousGamepad1.a){
                if(armState == ArmState.STRETCHED){
                    robotTop.setArmTurnPosition(0.71);
                }else if(armState == ArmState.TURNED){
                    robotTop.setArmTurnPosition(0.38);
                }
            }
            if(gamepad1.b && !previousGamepad1.b){
                if(armGrabbing){
                    robotTop.setArmGrabPosition(0);
                }else{
                    robotTop.setArmGrabPosition(0.53);
                }
            }
            if (gamepad1.dpad_up) {
                armSpinXPos = Math.min(1,armSpinXPos + 0.05);
                robotTop.setArmSpinXPosition(armSpinXPos);
            } else if (gamepad1.dpad_down) {
                armSpinXPos = Math.max(0, armSpinXPos - 0.05);
                robotTop.setArmSpinXPosition(armSpinXPos);
            }
            if (gamepad1.dpad_right) {
                armSpinYPos = Math.min(1,armSpinYPos + 0.05);
                robotTop.setArmSpinYPosition(armSpinYPos);
            } else if (gamepad1.dpad_left) {
                armSpinYPos = Math.max(0, armSpinYPos - 0.05);
                robotTop.setArmSpinYPosition(armSpinYPos);
            }

            //telemetry
            telemetry.addData("pos", liftPosition);
            telemetry.addData("state", liftState);
            telemetry.update();
            previousGamepad1.copy(gamepad1);
            sleep(50);
        }
    }
}
enum LiftState{
    BOTTOM,TOP
}
enum ArmState{
    IDLE,STRETCHED,TURNED
}