package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.RobotChassis;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;
import org.firstinspires.ftc.teamcode.hardware.RobotVision.RobotVisionAngle;

@TeleOp
public class ManualOpMode extends LinearOpMode {
    RobotChassis robotChassis;
    RobotTop robotTop;

    // servoName(port): positionRange[a, b]; defaultPos
    // control hub:
    // top(3): [0-idle, 0.62-out] up: 0.05; default: 0
    // container(2): [0.35-open, 1-close]; default: 1
    // lift(5): [0.2-open, 0.6-close];default: 0.6
    // --------------------------------------------
    // expansion hub:
    // stretch(1): [0-back, 0.3-out]; default: 0
    // spinX(2): [0, 1]; default: 0.5
    // turn(3): [0.38-back, 0.71-out]; default: 0.38
    // spinY(4): [0, 1]; default: 0.07
    // grab(5): [0.2-open, 0.83-close]; default: 0.2

    final int LIFT_TOP_POSITION = 1250;
    final int LIFT_HANGING_POSITION = 925;
    final int LIFT_BOTTOM_POSITION = 50;
    final double STRETCH_BACK_POSITION = 0.03;
    final double STRETCH_OUT_POSITION = 0.3;
    final double SPIN_X_DEFAULT_POSITION = 0.22;
    final double SPIN_X_HOVERING_POSITION = 0.53;
    final double SPIN_X_DOWN_POSITION = 0.58;
    final double SPIN_Y_DEFAULT_POSITION = 0.1;
    final double TURN_BACK_POSITION = 0.38;
    final double TURN_OUT_HOVERING_POSITION = 0.64;
    final double TURN_OUT_DOWN_POSITION = 0.68;
    final double GRAB_OPEN_POSITION = 0.2;
    final double GRAB_CLOSE_POSITION = 0.9;

    @Override
    public void runOpMode() {
        robotChassis = new RobotChassis(this);
        robotTop = new RobotTop(this);
        RobotVisionAngle robotVisionAngle = new RobotVisionAngle();
        robotVisionAngle.initialize(hardwareMap);
        Gamepad previousGamepad1 = new Gamepad();
        previousGamepad1.copy(gamepad1);
        Gamepad previousGamepad2 = new Gamepad();
        previousGamepad2.copy(gamepad2);

        int liftPosition;
        double armStretchPos = STRETCH_BACK_POSITION;
        double armTurnPos = TURN_BACK_POSITION;
        double armSpinXPos = SPIN_X_DEFAULT_POSITION;
        double armSpinYPos = SPIN_Y_DEFAULT_POSITION;
        boolean armGrabbing = false;
        boolean grabbingFlag = false;
        boolean topServoOut = false;
        boolean containerRelease = true;
        boolean backGrabOpen = false;
        double recognitionAngle = 0;
        LiftState liftState = LiftState.BOTTOM;
        ArmState armState = ArmState.IDLE;

        waitForStart();
        robotTop.setArmStretchPosition(armStretchPos);
        robotTop.setArmTurnPosition(armTurnPos);
        robotTop.setContainerServoPosition(0.35);
        robotTop.setLiftServoPosition(0.6);
        while (opModeIsActive()) {
            robotChassis.driveRobot(gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x);

            // robotLift
            liftPosition = robotTop.getLiftPosition();
//            if (liftState == LiftState.BOTTOM) {
//                if (gamepad1.y) {
//                    liftState = LiftState.GOING_UP;
//                }
//            } else if (liftState == LiftState.GOING_UP) {
//                robotTop.setTopServoPosition(0.05);
//                if (liftPosition >= LIFT_TOP_POSITION) {
//                    robotTop.setLeftPower(0);
//                    liftState = LiftState.TOP;
//                } else if (liftPosition >= LIFT_TOP_POSITION - 150) {
//                    robotTop.setLeftPower(0.3);
//                } else if (liftPosition <= LIFT_TOP_POSITION - 150) {
//                    robotTop.setLeftPower(0.7);
//                }
//            } else if (liftState == LiftState.TOP) {
//                if (gamepad1.y && liftPosition >= LIFT_BOTTOM_POSITION + 150) {
//                    robotTop.setLeftPower(-0.6);
//                    liftState = LiftState.GOING_DOWN;
//                }
//                if (liftPosition <= LIFT_TOP_POSITION) {
//                    robotTop.setLeftPower(0.3);
//                }
//            } else if (liftState == LiftState.GOING_DOWN) {
//                if (liftPosition <= LIFT_BOTTOM_POSITION) {
//                    robotTop.setLeftPower(0);
//                    robotTop.setTopServoPosition(0.05);
//                    sleep(500);
//                    liftState = LiftState.BOTTOM;
//                } else if (liftPosition <= LIFT_BOTTOM_POSITION + 150) {
//                    robotTop.setLeftPower(-0.2);
//                }
//            }
//            if (gamepad1.y && !previousGamepad1.y) {
//                if (armState == ArmState.IDLE) {
//                    robotChassis.stopMotor();
//                    armStretchPos = STRETCH_OUT_POSITION;
//                    robotTop.setArmStretchPosition(armStretchPos);
//                    robotTop.setArmSpinXPosition(SPIN_X_DEFAULT_POSITION + 0.2);
//                    sleep(1000);
//                    armState = ArmState.LOCKED;
//                    liftState = LiftState.GOING_UP;
//                } else if (armState == ArmState.LOCKED) {
//                    robotChassis.stopMotor();
//                    sleep(1500);
//                    armStretchPos = STRETCH_BACK_POSITION;
//                    armState = ArmState.WITHDRAWING;
//                }
//            }
            if (liftState == LiftState.BOTTOM && armState == ArmState.IDLE) {
                if (gamepad1.left_trigger != 0) {
                    armStretchPos = STRETCH_OUT_POSITION;
                    robotTop.setArmStretchPosition(armStretchPos);
                    armState = ArmState.STRETCHED;
                    sleep(1000);
                }
                if (gamepad1.right_trigger != 0) {
                    armStretchPos = STRETCH_OUT_POSITION;
                    robotTop.setArmStretchPosition(armStretchPos);
                    armState = ArmState.STRETCHED;
                    sleep(1000);
                }
            } else {
                if (gamepad1.left_trigger != 0) {
                    if(!topServoOut){
                        robotTop.setTopServoPosition(0.05);
                    }
                    robotTop.setLeftPower(-0.3);
                } else if (gamepad1.right_trigger != 0) {
                    if(!topServoOut){
                        robotTop.setTopServoPosition(0.05);
                    }
                    robotTop.setLeftPower(0.5);
                } else {
                    if(LIFT_HANGING_POSITION - 100 <= liftPosition && liftPosition <= LIFT_HANGING_POSITION){
                        robotTop.setLeftPower(0.03);
                    }else{
                        robotTop.setLeftPower(0);
                    }
                }
            }
            if (gamepad1.left_bumper && !previousGamepad1.left_bumper && liftPosition > 1100) {
                if (topServoOut) {
                    robotTop.setTopServoPosition(0.05);
                } else {
                    robotTop.setTopServoPosition(0.6);
                }
                topServoOut = !topServoOut;
            }
            if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
                if (containerRelease) {
                    robotTop.setContainerServoPosition(1);
                } else {
                    robotTop.setContainerServoPosition(0.35);
                }
                containerRelease = !containerRelease;
            }

            // robotArm
            if (gamepad1.x && !previousGamepad1.x) {
                if (armState == ArmState.IDLE) {
                    robotTop.setTopServoPosition(0);
                    armStretchPos = STRETCH_OUT_POSITION;
                    armState = ArmState.TURNING_OUT;
                    robotTop.setArmStretchPosition(armStretchPos);
                } else if (armState == ArmState.TURNED) {
                    armState = ArmState.TURNING_BACK;
                    robotTop.setArmSpinYPosition(SPIN_Y_DEFAULT_POSITION);
                }
            }
            if (armState == ArmState.WITHDRAWING) {
                if (armStretchPos <= STRETCH_BACK_POSITION) {
                    armState = ArmState.IDLE;
                } else {
                    armStretchPos -= 0.03;
                    robotTop.setArmStretchPosition(armStretchPos);
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
                    robotTop.setArmSpinXPosition(SPIN_X_DEFAULT_POSITION);
                    armState = ArmState.WITHDRAWING;
                } else {
                    armTurnPos -= 0.03;
                    robotTop.setArmTurnPosition(armTurnPos);
                }
            }
            if (armState == ArmState.STRETCHED) {
                if (gamepad1.x && !previousGamepad1.x) {
                    robotTop.setTopServoPosition(0);
                    armState = ArmState.WITHDRAWING;
                }
            }
            if (gamepad1.b && !previousGamepad1.b) {
                robotTop.setArmGrabPosition(GRAB_OPEN_POSITION);
                sleep(500);
                if (!armGrabbing && armState == ArmState.TURNED) {
                    robotTop.setArmTurnPosition(TURN_OUT_DOWN_POSITION);
                    robotTop.setArmSpinXPosition(SPIN_X_DOWN_POSITION);
                    grabbingFlag = true;
                } else {
                    robotTop.setArmGrabPosition(GRAB_CLOSE_POSITION);
                    armGrabbing = false;
                }
            }
            if (grabbingFlag && !gamepad1.b) {
                robotTop.setArmGrabPosition(GRAB_CLOSE_POSITION);
                sleep(500);
                robotTop.setArmTurnPosition(TURN_OUT_HOVERING_POSITION);
                robotTop.setArmSpinXPosition(SPIN_X_HOVERING_POSITION);
                armGrabbing = true;
                grabbingFlag = false;
            }
            robotTop.setArmStretchPosition(armStretchPos);
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
            if (gamepad1.a && !previousGamepad1.a) {
                if (backGrabOpen) {
                    robotTop.setLiftServoPosition(0.1);
                } else {
                    robotTop.setLiftServoPosition(0.6);
                }
                backGrabOpen = !backGrabOpen;
            }

            //vision
            if ((gamepad2.left_trigger != 0 && previousGamepad2.left_trigger == 0
            ) || (gamepad2.right_trigger != 0 && previousGamepad2.right_trigger == 0)) {
                //robotTop.setArmSpinYPosition(SPIN_Y_DEFAULT_POSITION);
                recognitionAngle = robotVisionAngle.getDetectedAngle();
                robotTop.setArmSpinYPosition(calculateSpinY(recognitionAngle));
            }

            // telemetry
            telemetry.addData("liftPos", liftPosition);
            telemetry.addData("state", liftState);
            telemetry.addData("ArmState", armState);
            telemetry.addData("XPos", armSpinXPos);
            telemetry.addData("YPos", armSpinYPos);
            telemetry.addData("angle", recognitionAngle);
            telemetry.update();
            previousGamepad1.copy(gamepad1);
            previousGamepad2.copy(gamepad2);
            sleep(50);
        }
    }

    protected double calculateSpinY(double angle) {
        if (angle < -180 || angle > 180) {
            return 0;
        }

        double adjustedAngle;
        if (angle <= 0) {
            adjustedAngle = -angle / 180 + 0.1;
        } else {
            adjustedAngle = 0.6 - angle / 180;
        }

        return Math.max(0, Math.min(adjustedAngle, 1)); // 确保值在0到1之间
    }


    enum LiftState {
        BOTTOM, GOING_UP, TOP, GOING_DOWN
    }

    enum ArmState {
        IDLE, WITHDRAWING, TURNING_OUT, TURNED, TURNING_BACK, LOCKED, STRETCHED
    }
}
