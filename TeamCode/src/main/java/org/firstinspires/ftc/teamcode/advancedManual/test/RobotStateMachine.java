package org.firstinspires.ftc.teamcode.advancedManual.test;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.advancedManual.TaskManager;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;

public class RobotStateMachine {
    enum ArmState{
        IDLE, WITHDRAWING, TURNING_OUT, TURNED, TURNING_BACK
    }
    enum LiftState {
        BOTTOM, GOING_UP, TOP, GOING_DOWN
    }
    RobotTop robotTop;
    ArmState armState;
    LiftState liftState;
    Gamepad gamepad1;
    Gamepad previousGamepad1;
    TaskManager taskMgr;

    final double STRETCH_BACK_POSITION = 0.03;
    final double STRETCH_OUT_POSITION = 0.3;
    final double SPIN_X_DEFAULT_POSITION = 0.22;
    final double SPIN_X_HOVERING_POSITION = 0.53;
    final double SPIN_X_DOWN_POSITION = 0.58;
    final double SPIN_Y_DEFAULT_POSITION = 0.1;
    final double TURN_BACK_POSITION = 0.38;
    final double TURN_OUT_HOVERING_POSITION = 0.64;
    final double TURN_OUT_DOWN_POSITION = 0.71;
    final double GRAB_OPEN_POSITION = 0.2;
    final double GRAB_CLOSE_POSITION = 0.9;
    final double CONTAINER_OPEN_POSITION = 0.35;
    final double CONTAINER_CLOSE_POSITION = 1;
    final double LIFT_SERVO_CLOSE = 0.6;
    final double LIFT_SERVO_OPEN = 0.2;

    double armStretchPos;
    double armTurnPos;
    double armSpinXPos;
    double armSpinYPos;
    public RobotStateMachine(RobotTop robotTop){
        this.robotTop = robotTop;
        this.armState = ArmState.IDLE;
        this.liftState = LiftState.BOTTOM;
        this.taskMgr = new TaskManager();
    }
    protected void init(){
        int liftPosition;
        double armStretchPos = STRETCH_BACK_POSITION;
        double armTurnPos = TURN_BACK_POSITION;
        double armSpinXPos = SPIN_X_DEFAULT_POSITION;
        double armSpinYPos = SPIN_Y_DEFAULT_POSITION;
        boolean armGrabbing = false;
        boolean grabbingFlag = false;
        boolean topServoState = false;
        boolean containerRelease = false;
        double recognitionAngle = 0;
    }

    private void initializeRobotPositions() {
        // robotTop.setArmStretchPosition(armStretchPos);
        robotTop.setArmLeftTurnPosition(armTurnPos);
        robotTop.setContainerServoPosition(CONTAINER_OPEN_POSITION);
        robotTop.setLiftServoPosition(LIFT_SERVO_CLOSE);
    }

    public void update(){
        switch (armState){
            case IDLE:
                handleArmIdle();
            case TURNING_OUT:
                handleArmTurningOut();
            case TURNED:
                handleArmTurned();
            case TURNING_BACK:
                handleArmTurningBack();
            case WITHDRAWING:
                handleWithdrawingState();
        }
    }
    protected void handleArmIdle(){
        if(gamepad1.x && !previousGamepad1.x){
            armStretchPos = STRETCH_OUT_POSITION;
            // robotTop.setArmStretchPosition(armStretchPos);
            robotTop.setTopServoPosition(0);
        }
    }
    protected void handleArmTurningOut(){}
    protected void handleArmTurned(){}
    protected void handleArmTurningBack(){}
    protected void handleWithdrawingState(){}
}
