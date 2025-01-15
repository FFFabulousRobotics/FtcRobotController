package org.firstinspires.ftc.teamcode.advancedManual;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.RobotChassis;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;

public class ArmStateMachine {
    enum ArmState{
        IDLE, WITHDRAWING, TURNING_OUT, TURNED, TURNING_BACK
    }
    RobotTop robotTop;
    RobotChassis robotChassis;
    ArmState armState;
    Gamepad gamepad1;
    Gamepad previousGamepad1;
    Gamepad gamepad2;
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

    double armStretchPos;
    double armTurnPos;
    double armSpinXPos;
    double armSpinYPos;
    public ArmStateMachine(RobotTop robotTop, RobotChassis robotChassis){
        this.robotTop = robotTop;
        this.robotChassis = robotChassis;
        this.armState = ArmState.IDLE;
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
    public void receiveGamepad(Gamepad gamepad1, Gamepad previousGamepad1,
                                Gamepad gamepad2){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.previousGamepad1 = previousGamepad1;
    }

    public void update(){
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
    }
    protected void handleIdleState(){
    }
    protected void handleTurningOutState(){}
    protected void handleTurnedState(){}
    protected void handleTurningBackState(){}
    protected void handleWithdrawingState(){}
}
