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

    // servoName(port): positionRange[a, b]; defaultPos
    // control hub:
    // top(3):
    // container(4): [0.35-open, 1-close]; default: 1
    // lift(5): [0.2-open, 0.6-close];default: 0.6
    // --------------------------------------------
    // expansion hub:
    // stretch(1): [0-back, 0.3-out]; default: 0
    // spinX(2): [0, 1]; default: 0.5
    // turn(3): [0.38-back, 0.71-out]; default: 0.38
    // spinY(4): [0, 1]; default: 0.07
    // grab(5): [0.2-open, 0.83-close]; default: 0.2

    final int LIFT_TOP_POSITION = 1250;
    final int LIFT_BOTTOM_POSITION = 50;
    final double STRETCH_BACK_POSITION = 0.03;
    final double STRETCH_OUT_POSITION = 0.3;
    final double SPIN_X_DEFAULT_POSITION = 0.5;
    //TODO: test spinX hovering position
    final double SPIN_X_HOVERING_POSITION = 0.53;
    //TODO: test the spinX down position
    final double SPIN_X_DOWN_POSITION = 0.58;
    final double SPIN_Y_DEFAULT_POSITION = 0.1;
    final double TURN_BACK_POSITION = 0.38;
    //TODO: test the hovering position
    final double TURN_OUT_HOVERING_POSITION = 0.64;
    //TODO: test the grabbing down position
    final double TURN_OUT_DOWN_POSITION = 0.71;
    final double GRAB_OPEN_POSITION = 0.2;
    final double GRAB_CLOSE_POSITION = 0.9;

    @Override
    public void runOpMode() {
        robotChassis = new RobotChassis(this);
        robotTop = new RobotTop(this);
        Gamepad previousGamepad1 = new Gamepad();
        previousGamepad1.copy(gamepad1);

        int liftPosition;
        double armStretchPos = STRETCH_BACK_POSITION;
        double armTurnPos = TURN_BACK_POSITION;
        double armSpinXPos = SPIN_X_DEFAULT_POSITION;
        double armSpinYPos = SPIN_Y_DEFAULT_POSITION;
        boolean armGrabbing = false;
        LiftState liftState = LiftState.BOTTOM;
        ArmState armState = ArmState.IDLE;

        waitForStart();
        while (opModeIsActive()) {
            robotChassis.driveRobot(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            // robotLift
            liftPosition = robotTop.getLiftPosition();
            if (liftState == LiftState.BOTTOM) {
                if (gamepad1.y && liftPosition <= LIFT_TOP_POSITION - 150) {
                    robotTop.setLeftPower(0.7);
                } else if (liftPosition >= LIFT_TOP_POSITION) {
                    robotTop.setLeftPower(0);
                    liftState = LiftState.TOP;
                } else if (liftPosition >= LIFT_TOP_POSITION - 150) {
                    robotTop.setLeftPower(0.3);
                }
            } else if (liftState == LiftState.TOP) {
                if (gamepad1.y && liftPosition >= LIFT_BOTTOM_POSITION + 150) {
                    robotTop.setLeftPower(-0.6);
                } else if (liftPosition <= LIFT_BOTTOM_POSITION) {
                    robotTop.setLeftPower(0);
                    liftState = LiftState.BOTTOM;
                } else if (liftPosition <= LIFT_BOTTOM_POSITION + 150) {
                    robotTop.setLeftPower(-0.2);
                }
            }

            // robotArm
            if (gamepad1.x && !previousGamepad1.x) {
                if (armState == ArmState.IDLE) {
                    armStretchPos = STRETCH_OUT_POSITION;
                    armState = ArmState.STRETCHED;
                    robotTop.setArmStretchPosition(armStretchPos);
                } else if (armState == ArmState.STRETCHED) {
                    armState = ArmState.WITHDRAWING;
                }
            }
            if (armState == ArmState.WITHDRAWING) {
                if (armStretchPos <= STRETCH_BACK_POSITION) {
                    armState = ArmState.IDLE;
                } else {
                    armStretchPos -= 0.04;
                    robotTop.setArmStretchPosition(armStretchPos);
                }
            }
            if (gamepad1.a && !previousGamepad1.a) {
                if (armState == ArmState.STRETCHED) {
                    armState = ArmState.TURNING_OUT;
                } else if (armState == ArmState.TURNED) {
                    armState = ArmState.TURNING_BACK;
                }
            }
            if (armState == ArmState.TURNING_OUT) {
                if (armTurnPos >= TURN_OUT_HOVERING_POSITION - 0.05) {
                    armTurnPos = TURN_OUT_HOVERING_POSITION;
                    robotTop.setArmTurnPosition(armTurnPos);
                    robotTop.setArmSpinXPosition(SPIN_X_HOVERING_POSITION);
                    armState = ArmState.TURNED;
                } else {
                    armTurnPos += 0.03;
                    robotTop.setArmTurnPosition(armTurnPos);
                }
            }
            if (armState == ArmState.TURNING_BACK) {
                if (armTurnPos <= TURN_BACK_POSITION + 0.05) {
                    armTurnPos = TURN_BACK_POSITION;
                    robotTop.setArmTurnPosition(armTurnPos);
                    armState = ArmState.STRETCHED;
                } else {
                    armTurnPos -= 0.03;
                    robotTop.setArmTurnPosition(armTurnPos);
                }
            }
            if (gamepad1.b && !previousGamepad1.b) {
                robotTop.setArmGrabPosition(GRAB_OPEN_POSITION);
                sleep(500);
                if (!armGrabbing) {
                    robotTop.setArmTurnPosition(TURN_OUT_DOWN_POSITION);
                    robotTop.setArmSpinXPosition(SPIN_X_DOWN_POSITION);
                    sleep(200);
                    robotTop.setArmGrabPosition(GRAB_CLOSE_POSITION);
                    sleep(500);
                    robotTop.setArmTurnPosition(TURN_OUT_HOVERING_POSITION);
                    armGrabbing = true;
                    telemetry.addData("grab", 1);
                } else {
                    robotTop.setArmGrabPosition(GRAB_CLOSE_POSITION);
                    armGrabbing = false;
                }
            }
            if (gamepad1.dpad_up) {
                armSpinXPos = Math.min(1, armSpinXPos + 0.02);
                robotTop.setArmSpinXPosition(armSpinXPos);
            } else if (gamepad1.dpad_down) {
                armSpinXPos = Math.max(0, armSpinXPos - 0.02);
                robotTop.setArmSpinXPosition(armSpinXPos);
            }
            if (gamepad1.dpad_right) {
                armSpinYPos = Math.min(1, armSpinYPos + 0.05);
                robotTop.setArmSpinYPosition(armSpinYPos);
            } else if (gamepad1.dpad_left) {
                armSpinYPos = Math.max(0, armSpinYPos - 0.05);
                robotTop.setArmSpinYPosition(armSpinYPos);
            }

            // telemetry
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

enum LiftState {
    BOTTOM, TOP
}

enum ArmState {
    IDLE, WITHDRAWING, STRETCHED, TURNING_OUT, TURNED, TURNING_BACK
}