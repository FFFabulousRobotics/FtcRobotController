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
        double armStretchPos = 0;
        double armTurnPos = 0;
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
                    armStretchPos = 0.3;
                    armState = ArmState.STRETCHED;
                    robotTop.setArmStretchPosition(armStretchPos);
                }else if(armState == ArmState.STRETCHED){
                    armState = ArmState.WITHDRAWING;
                }
            }
            if(armState == ArmState.WITHDRAWING){
                if(armStretchPos <= 0){
                    armState = ArmState.IDLE;
                }else{
                    armStretchPos -= 0.04;
                    robotTop.setArmStretchPosition(armStretchPos);
                }
            }
            if(gamepad1.a && !previousGamepad1.a){
                if(armState == ArmState.STRETCHED){
                    armState = ArmState.TURNING_OUT;
                }else if(armState == ArmState.TURNED){
                    armState = ArmState.TURNING_BACK;
                }
            }
            if(armState == ArmState.TURNING_OUT){
                if(armTurnPos >= 0.68){
                    armTurnPos = 0.71;
                    robotTop.setArmTurnPosition(armTurnPos);
                    armState = ArmState.TURNED;
                }else{
                    armTurnPos += 0.03;
                    robotTop.setArmTurnPosition(armTurnPos);
                }
            }
            if(armState == ArmState.TURNING_BACK){
                if(armTurnPos <= 0.42){
                    armTurnPos = 0.38;
                    robotTop.setArmTurnPosition(armTurnPos);
                    armState = ArmState.STRETCHED;
                }else{
                    armTurnPos -= 0.03;
                    robotTop.setArmTurnPosition(armTurnPos);
                }
            }
            if(gamepad1.b && !previousGamepad1.b){
                if(armGrabbing){
                    robotTop.setArmGrabPosition(0.73);
                    armGrabbing = false;
                }else{
                    robotTop.setArmGrabPosition(0.2);
                    armGrabbing = true;
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
            telemetry.addData("liftPos", liftPosition);
            telemetry.addData("state", liftState);
            telemetry.addData("ArmState", armState);
            telemetry.addData("XPos", armSpinXPos);
            telemetry.addData("YPos", armSpinYPos);
            telemetry.addData("grab", armGrabbing);
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
    IDLE,WITHDRAWING,STRETCHED,TURNING_OUT,TURNED,TURNING_BACK
}