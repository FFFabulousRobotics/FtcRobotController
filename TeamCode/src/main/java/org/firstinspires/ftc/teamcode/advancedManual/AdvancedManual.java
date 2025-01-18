package org.firstinspires.ftc.teamcode.advancedManual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.RobotChassis;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;

// The ManualOp currently used is a piece of shit.
// I'll write a better one here (if the time is enough).
@TeleOp
public class AdvancedManual extends LinearOpMode {
    enum ArmState{
        IDLE, WITHDRAWING, TURNING_OUT, TURNED, TURNING_BACK, LOCKED
    }
    enum LiftState{
        DISABLED, RUNNING
    }
    RobotTop robotTop;
    RobotChassis robotChassis;
    ArmState armState;
    LiftState liftState;
    Gamepad previousGamepad1;

    //TODO: test the constants
    final int STRETCH_BACK_POSITION = 70;
    final int STRETCH_OUT_POSITION = 1500;
    final double SPIN_DEFAULT_POSITION = 0.3;
    final double SPIN_HOVERING_POSITION = 1;
    final double SPIN_DOWN_POSITION = 0;
    final double TURN_BACK_POSITION = 0;
    final double TURN_HOVERING_POSITION = 0.4;
    final double TURN_DOWN_POSITION = 0.45;
    final double GRAB_OPEN_POSITION = 0.4;
    final double GRAB_CLOSE_POSITION = 0.92;
    final double TOP_BACK = 0.03;
    final double TOP_OUT = 0.66;

    boolean isGrabbing;
    boolean topServoOut;
    boolean backGrabOpen;
    int liftPosition;
    double targetTurnPosition;
    double currentTurnPosition;
    @Override
    public void runOpMode(){
        previousGamepad1 = new Gamepad();
        previousGamepad1.copy(gamepad1);
        this.robotTop = new RobotTop(this);
        this.robotChassis = new RobotChassis(this);
        this.armState = ArmState.IDLE;
        this.liftState = LiftState.DISABLED;
        this.isGrabbing = false;
        this.topServoOut = false;
        this.liftPosition = 0;
        this.backGrabOpen = false;
        this.targetTurnPosition = TURN_BACK_POSITION;
        waitForStart();
        robotTop.setTurnPosition(TURN_BACK_POSITION);
        robotTop.setArmLeftSpinPosition(SPIN_DEFAULT_POSITION);
        robotTop.setArmRightSpinPosition(SPIN_DEFAULT_POSITION);
        int armStretchStartPos = robotTop.getArmStretchPosition();
        telemetry.addData("aSTARTpos", armStretchStartPos);
        telemetry.update();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robotChassis.driveRobot(gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x);
            if (gamepad1.a && !previousGamepad1.a) {
                if (backGrabOpen) {
                    robotTop.setLiftServoPosition(0.1);
                } else {
                    robotTop.setLiftServoPosition(0.6);
                }
                backGrabOpen = !backGrabOpen;
            }
            switch (armState){
                case IDLE:
                    handleIdleState();
                    break;
                case TURNING_OUT:
                    handleTurningOutState();
                    break;
                case TURNED:
                    handleTurnedState();
                    break;
                case TURNING_BACK:
                    handleTurningBackState();
                    break;
                case WITHDRAWING:
                    handleWithdrawingState();
                    break;
                case LOCKED:
                    handleLockedState();
                    break;
            }
            switch (liftState){
                case DISABLED:
                    handleDisabledState();
                    break;
                case RUNNING:
                    handleRunningState();
                    break;
            }
            telemetry.addData("arm",armState);
            telemetry.addData("lift",liftState);
            telemetry.addData("armPos",robotTop.getTurnPosition());
            telemetry.addData("armStretchPosition", robotTop.getArmStretchPosition());
            telemetry.update();
            previousGamepad1.copy(gamepad1);
            sleep(10);
        }

    }
    protected void handleIdleState(){
        if(gamepad1.x && !previousGamepad1.x){
            robotTop.setStretchPower(0.9);
            targetTurnPosition = TURN_HOVERING_POSITION;
//            robotTop.setTurnPosition(TURN_HOVERING_POSITION);
            robotTop.setArmLeftSpinPosition(SPIN_HOVERING_POSITION);
            robotTop.setArmRightSpinPosition(SPIN_HOVERING_POSITION);
            armState = ArmState.TURNING_OUT;
        }
        if(gamepad1.b && !previousGamepad1.b){
            if(isGrabbing){
                robotTop.setArmGrabPosition(GRAB_CLOSE_POSITION);
            }else {
                robotTop.setArmGrabPosition(GRAB_OPEN_POSITION);
            }
        }
        if(liftState == LiftState.RUNNING){
            liftState = LiftState.DISABLED;
        }
        if(gamepad1.dpad_up){
            robotTop.setArmLeftSpinPosition(Math.min(1, robotTop.getArmLeftSpinPosition() + 0.008));
            robotTop.setArmRightSpinPosition(Math.min(1, robotTop.getArmRightSpinPosition() + 0.008));
        }
        if(gamepad1.dpad_down){
            robotTop.setArmLeftSpinPosition(Math.max(0, robotTop.getArmLeftSpinPosition() - 0.008));
            robotTop.setArmRightSpinPosition(Math.max(0, robotTop.getArmRightSpinPosition() - 0.008));
        }
        if(gamepad1.dpad_left){
            robotTop.setArmLeftSpinPosition(Math.min(1, robotTop.getArmLeftSpinPosition() + 0.008));
            robotTop.setArmRightSpinPosition(Math.max(0, robotTop.getArmRightSpinPosition() - 0.008));
        }
        if(gamepad1.dpad_right){
            robotTop.setArmLeftSpinPosition(Math.max(0, robotTop.getArmLeftSpinPosition() - 0.008));
            robotTop.setArmRightSpinPosition(Math.min(1, robotTop.getArmRightSpinPosition() + 0.008));
        }
    }

    protected void handleTurningOutState(){
        if(robotTop.getArmStretchPosition() >= STRETCH_OUT_POSITION){
            robotTop.setStretchPower(0);
            armState = ArmState.TURNED;
        }
        currentTurnPosition = robotTop.getTurnPosition();
        if(currentTurnPosition > targetTurnPosition + 0.005){
            robotTop.setTurnPosition(currentTurnPosition - 0.005);
        } else if (currentTurnPosition < targetTurnPosition - 0.005) {
            robotTop.setTurnPosition(currentTurnPosition + 0.005);
        }else{
            robotTop.setTurnPosition(targetTurnPosition);
        }
//        if(robotTop.getTurnPosition() >= TURN_DOWN_POSITION){
//            robotTop.setStretchPower(0);
//            armState = ArmState.TURNED;
//        }
//        robotTop.setTurnPosition(robotTop.getTurnPosition() + 0.005);
    }
    protected void handleTurnedState(){
        if(gamepad1.x && !previousGamepad1.x){
            robotTop.setStretchPower(-0.9);
            targetTurnPosition = TURN_BACK_POSITION;
            // robotTop.setTurnPosition(TURN_BACK_POSITION);
            robotTop.setArmLeftSpinPosition(SPIN_DEFAULT_POSITION);
            robotTop.setArmRightSpinPosition(SPIN_DEFAULT_POSITION);
            armState = ArmState.WITHDRAWING;
        }
        if(gamepad1.b && !previousGamepad1.b){
            if(isGrabbing){
                robotTop.setTurnPosition(TURN_DOWN_POSITION);
                sleep(500);
                robotTop.setArmGrabPosition(GRAB_CLOSE_POSITION);
                sleep(500);
                robotTop.setTurnPosition(TURN_HOVERING_POSITION);
                isGrabbing = !isGrabbing;
            }else {
                robotTop.setArmGrabPosition(GRAB_OPEN_POSITION);
                isGrabbing = !isGrabbing;
            }
        }
        if(gamepad1.dpad_up){
            robotTop.setArmLeftSpinPosition(Math.min(1, robotTop.getArmLeftSpinPosition() + 0.008));
            robotTop.setArmRightSpinPosition(Math.min(1, robotTop.getArmRightSpinPosition() + 0.008));
        }
        if(gamepad1.dpad_down){
            robotTop.setArmLeftSpinPosition(Math.max(0, robotTop.getArmLeftSpinPosition() - 0.008));
            robotTop.setArmRightSpinPosition(Math.max(0, robotTop.getArmRightSpinPosition() - 0.008));
        }
        if(gamepad1.dpad_left){
            robotTop.setArmLeftSpinPosition(Math.min(1, robotTop.getArmLeftSpinPosition() + 0.008));
            robotTop.setArmRightSpinPosition(Math.max(0, robotTop.getArmRightSpinPosition() - 0.008));
        }
        if(gamepad1.dpad_right){
            robotTop.setArmLeftSpinPosition(Math.max(0, robotTop.getArmLeftSpinPosition() - 0.008));
            robotTop.setArmRightSpinPosition(Math.min(1, robotTop.getArmRightSpinPosition() + 0.008));
        }
    }
    protected void handleTurningBackState(){
        //if necessary
    }
    protected void handleWithdrawingState(){
        if(robotTop.getArmStretchPosition() <= STRETCH_BACK_POSITION){
            robotTop.setStretchPower(0);
            armState = ArmState.IDLE;
        }
        currentTurnPosition = robotTop.getTurnPosition();
        if(currentTurnPosition > targetTurnPosition + 0.005){
            robotTop.setTurnPosition(currentTurnPosition - 0.005);
        } else if (currentTurnPosition < targetTurnPosition - 0.005) {
            robotTop.setTurnPosition(currentTurnPosition + 0.005);
        }else{
            robotTop.setTurnPosition(targetTurnPosition);
        }
//        if(robotTop.getTurnPosition() <= TURN_BACK_POSITION){
//            robotTop.setStretchPower(0);
//            armState = ArmState.IDLE;
//        }
//        robotTop.setTurnPosition(robotTop.getTurnPosition() - 0.005);
    }

    protected void handleLockedState(){
        if(gamepad1.y && !previousGamepad1.y){
            liftState = LiftState.DISABLED;
            robotTop.setStretchPower(-0.8);
            armState = ArmState.WITHDRAWING;
        }
    }

    protected void handleDisabledState(){
        if(armState != ArmState.IDLE || armState != ArmState.LOCKED){
            liftState = LiftState.RUNNING;
        }
        if(gamepad1.right_trigger != 0 || gamepad1.left_trigger != 0){
            robotTop.setStretchPower(0.8);
            sleep(700);
            robotTop.setStretchPower(0);
            armState = ArmState.LOCKED;
            liftState = LiftState.RUNNING;
        }
    }
    protected void handleRunningState(){
        if(gamepad1.right_trigger != 0){
            robotTop.setLeftPower(0.5);
            robotTop.setLiftTargetPos(robotTop.getLiftPosition());
        } else if (gamepad1.left_trigger != 0) {
            robotTop.setLeftPower(-0.5);
            robotTop.setLiftTargetPos(robotTop.getLiftPosition());
        }else{
            if(robotTop.getLiftPosition() >= 200){
                robotTop.updateLiftPID();
            }else{
                robotTop.setLeftPower(0);
            }
        }
        if (gamepad1.left_bumper && !previousGamepad1.left_bumper) {
            if (topServoOut) {
                robotTop.setTopServoPosition(TOP_BACK);
            } else {
                robotTop.setTopServoPosition(TOP_OUT);
            }
            topServoOut = !topServoOut;
        }
    }
}
