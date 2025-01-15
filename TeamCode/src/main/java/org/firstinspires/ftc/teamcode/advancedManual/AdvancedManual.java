package org.firstinspires.ftc.teamcode.advancedManual;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.RobotChassis;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;

@Disabled
// The ManualOp currently used is a piece of shit.
// I'll write a better one here (if the time is enough).
public class AdvancedManual extends LinearOpMode {
    enum ArmState{
        IDLE, WITHDRAWING, TURNING_OUT, TURNED, TURNING_BACK
    }
    RobotTop robotTop;
    RobotChassis robotChassis;
    ArmStateMachine.ArmState armState;
    Gamepad gamepad1;
    Gamepad previousGamepad1;
    Gamepad gamepad2;

    //TODO: test the constants
    final int STRETCH_BACK_POSITION = 0;
    final int STRETCH_OUT_POSITION = 0;
    final double SPIN_DEFAULT_POSITION = 0;
    final double SPIN_HOVERING_POSITION = 0;
    final double SPIN_DOWN_POSITION = 0;
    final double TURN_BACK_POSITION = 0;
    final double TURN_HOVERING_POSITION = 0;
    final double TURN_DOWN_POSITION = 0;
    final double GRAB_OPEN_POSITION = 0;
    final double GRAB_CLOSE_POSITION = 0;

    boolean isGrabbing;
    @Override
    public void runOpMode(){
        Gamepad previousGamepad1 = new Gamepad();
        previousGamepad1.copy(gamepad1);
        this.robotTop = new RobotTop(this);
        this.robotChassis = new RobotChassis(this);
        this.armState = ArmStateMachine.ArmState.IDLE;
        this.isGrabbing = false;
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robotChassis.driveRobot(gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x);
            switch (armState){
                case IDLE:
                    handleIdleState();
                case TURNING_OUT:
                    handleTurningOutState();
                case TURNED:
                    handleTurnedState();
                case TURNING_BACK:
                    handleTurningBackState();
                case WITHDRAWING:
                    handleWithdrawingState();
            }
            this.previousGamepad1.copy(gamepad1);
            sleep(50);
        }

    }
    protected void handleIdleState(){
        if(gamepad1.x && !previousGamepad1.x){
            robotTop.setArmStretchPosition(STRETCH_OUT_POSITION);
            robotTop.setArmLeftTurnPosition(TURN_HOVERING_POSITION);
            robotTop.setArmRightTurnPosition(TURN_HOVERING_POSITION);
            robotTop.setArmLeftSpinPosition(SPIN_HOVERING_POSITION);
            robotTop.setArmRightSpinPosition(SPIN_HOVERING_POSITION);
            armState = ArmStateMachine.ArmState.TURNED;
        }
    }
    protected void handleTurningOutState(){
        // if necessary
    }
    protected void handleTurnedState(){
        if(gamepad1.x && !previousGamepad1.x){
            robotTop.setArmStretchPosition(STRETCH_BACK_POSITION);
            robotTop.setArmLeftTurnPosition(TURN_BACK_POSITION);
            robotTop.setArmRightTurnPosition(TURN_BACK_POSITION);
            robotTop.setArmLeftSpinPosition(SPIN_DEFAULT_POSITION);
            robotTop.setArmRightSpinPosition(SPIN_DEFAULT_POSITION);
            armState = ArmStateMachine.ArmState.IDLE;
        }
        if(gamepad1.b && !previousGamepad1.b){
            if(isGrabbing){
                robotTop.setArmGrabPosition(GRAB_CLOSE_POSITION);
            }else {
                robotTop.setArmGrabPosition(GRAB_OPEN_POSITION);
            }
        }
        if(gamepad1.dpad_up){
            robotTop.setArmLeftSpinPosition(Math.min(1, robotTop.getArmLeftSpinPosition() + 0.002));
            robotTop.setArmRightSpinPosition(Math.min(1, robotTop.getArmRightSpinPosition() + 0.002));
        }
        if(gamepad1.dpad_down){
            robotTop.setArmLeftSpinPosition(Math.max(0, robotTop.getArmLeftSpinPosition() - 0.002));
            robotTop.setArmRightSpinPosition(Math.max(0, robotTop.getArmRightSpinPosition() - 0.002));
        }
        if(gamepad1.dpad_left){
            robotTop.setArmLeftSpinPosition(Math.min(1, robotTop.getArmLeftSpinPosition() + 0.002));
            robotTop.setArmRightSpinPosition(Math.max(0, robotTop.getArmRightSpinPosition() - 0.002));
        }
        if(gamepad1.dpad_right){
            robotTop.setArmLeftSpinPosition(Math.max(0, robotTop.getArmLeftSpinPosition() - 0.002));
            robotTop.setArmRightSpinPosition(Math.min(1, robotTop.getArmRightSpinPosition() + 0.002));
        }
    }
    protected void handleTurningBackState(){
        //if necessary
    }
    protected void handleWithdrawingState(){
        //if necessary
    }
}
