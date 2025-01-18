package org.firstinspires.ftc.teamcode.previousCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.RobotChassis;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;

@Disabled
@TeleOp
public class PreviousManual extends LinearOpMode {
    enum ArmState {
        IDLE, WITHDRAWING, TURNING_OUT, TURNED, TURNING_BACK, LOCKED
    }

    enum LiftState {
        DISABLED, RUNNING
    }

    RobotTop robotTop;
    RobotChassis robotChassis;
    ArmState armState;
    LiftState liftState;
    Gamepad previousGamepad1;

    final int STRETCH_BACK_POSITION = 70;
    final int STRETCH_OUT_POSITION = 1500;
    final double SPIN_DEFAULT_POSITION = 0.3;
    final double SPIN_HOVERING_POSITION = 1;
    final double SPIN_DOWN_POSITION = 0;
    final double TURN_BACK_POSITION = 0.5;
    final double TURN_HOVERING_POSITION = 0.8;
    final double TURN_DOWN_POSITION = 0.85;
    final double GRAB_OPEN_POSITION = 0.4;
    final double GRAB_CLOSE_POSITION = 0.92;
    final double TOP_BACK = 0.03;
    final double TOP_OUT = 0.66;

    boolean isGrabbing;
    boolean topServoOut;
    boolean backGrabOpen;
    int liftPosition;
    double turnPosition;

    @Override
    public void runOpMode() {
        previousGamepad1 = new Gamepad();
        previousGamepad1.copy(gamepad1);
        this.robotTop = new RobotTop(this);
        this.robotChassis = new RobotChassis(this);
        this.armState = ArmState.IDLE;
        this.liftState = LiftState.DISABLED;
        this.isGrabbing = false;
        this.topServoOut = false;
        this.backGrabOpen = false;
        this.liftPosition = 0;
        waitForStart();
        robotTop.setTurnPosition(TURN_BACK_POSITION);
        robotTop.setArmLeftSpinPosition(SPIN_DEFAULT_POSITION);
        robotTop.setArmRightSpinPosition(SPIN_DEFAULT_POSITION);
        while (opModeIsActive()) {
            robotChassis.driveRobot(gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x);

            // robotLift
            liftPosition = robotTop.getLiftPosition();
            turnPosition = robotTop.getTurnPosition();

            if (gamepad1.x && !gamepad1.left_bumper) {
                robotTop.setStretchPower(0.5);
            } else if (gamepad1.x && gamepad1.left_bumper) {
                robotTop.setStretchPower(-0.5);
            } else {
                robotTop.setStretchPower(0);
            }

            if (gamepad1.y && !gamepad1.left_bumper) {
                turnPosition += 0.005;
            } else if (gamepad1.y && gamepad1.left_bumper) {
                turnPosition -= 0.005;
            }
            robotTop.setTurnPosition(turnPosition);

            if (gamepad1.b && !previousGamepad1.b) {
                if (isGrabbing) {
                    robotTop.setArmGrabPosition(GRAB_CLOSE_POSITION);
                } else {
                    robotTop.setArmGrabPosition(GRAB_OPEN_POSITION);
                }
                isGrabbing = !isGrabbing;
            }

            if (gamepad1.dpad_up) {
                robotTop.setArmLeftSpinPosition(Math.min(1, robotTop.getArmLeftSpinPosition() + 0.005));
                robotTop.setArmRightSpinPosition(Math.min(1, robotTop.getArmRightSpinPosition() + 0.005));
            }
            if (gamepad1.dpad_down) {
                robotTop.setArmLeftSpinPosition(Math.max(0, robotTop.getArmLeftSpinPosition() - 0.005));
                robotTop.setArmRightSpinPosition(Math.max(0, robotTop.getArmRightSpinPosition() - 0.005));
            }
            if (gamepad1.dpad_left) {
                robotTop.setArmLeftSpinPosition(Math.min(1, robotTop.getArmLeftSpinPosition() + 0.005));
                robotTop.setArmRightSpinPosition(Math.max(0, robotTop.getArmRightSpinPosition() - 0.005));
            }
            if (gamepad1.dpad_right) {
                robotTop.setArmLeftSpinPosition(Math.max(0, robotTop.getArmLeftSpinPosition() - 0.005));
                robotTop.setArmRightSpinPosition(Math.min(1, robotTop.getArmRightSpinPosition() + 0.005));
            }

            if (gamepad1.right_trigger != 0) {
                robotTop.setLeftPower(0.5);
                robotTop.setLiftTargetPos(robotTop.getLiftPosition());
            } else if (gamepad1.left_trigger != 0) {
                robotTop.setLeftPower(-0.5);
                robotTop.setLiftTargetPos(robotTop.getLiftPosition());
            } else {
                if (robotTop.getLiftPosition() >= 200) {
                    robotTop.updateLiftPID();
                } else {
                    robotTop.setLeftPower(0);
                }
            }

            if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
                if (topServoOut) {
                    robotTop.setTopServoPosition(TOP_BACK);
                } else {
                    robotTop.setTopServoPosition(TOP_OUT);
                }
                topServoOut = !topServoOut;
            }

            if (gamepad1.a && !previousGamepad1.a) {
                if (backGrabOpen) {
                    robotTop.setLiftServoPosition(0.1);
                } else {
                    robotTop.setLiftServoPosition(0.6);
                }
                backGrabOpen = !backGrabOpen;
            }

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


//            if (liftState == LiftState.BOTTOM && armState == ArmState.IDLE) {
//                if (gamepad1.left_trigger != 0) {
//                    armStretchPos = STRETCH_OUT_POSITION;
//                    robotTop.setArmStretchPosition(armStretchPos);
//                    armState = ArmState.STRETCHED;
//                    sleep(1000);
//                }
//                if (gamepad1.right_trigger != 0) {
//                    armStretchPos = STRETCH_OUT_POSITION;
//                    robotTop.setArmStretchPosition(armStretchPos);
//                    armState = ArmState.STRETCHED;
//                    sleep(1000);
//                }
//            } else {
//                if (gamepad1.left_trigger != 0) {
//                    if (!topServoOut) {
//                        robotTop.setTopServoPosition(0.05);
//                    }
//                    robotTop.setLeftPower(-0.5);
//                } else if (gamepad1.right_trigger != 0) {
//                    if (!topServoOut) {
//                        robotTop.setTopServoPosition(0.05);
//                    }
//                    robotTop.setLeftPower(0.5);
//                } else {
//                    if (LIFT_TOP_POSITION - 150 <= liftPosition && liftPosition <= LIFT_TOP_POSITION) {
//                        robotTop.setLeftPower(0.2);
//                    } else {
//                        robotTop.setLeftPower(0);
//                    }
//                }
//            }
//            if (gamepad1.left_bumper && !previousGamepad1.left_bumper && liftPosition > 1100) {
//                if (topServoOut) {
//                    robotTop.setTopServoPosition(0.05);
//                } else {
//                    robotTop.setTopServoPosition(0.6);
//                }
//                topServoOut = !topServoOut;
//            }
//            if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
//                if (containerRelease) {
//                    robotTop.setContainerServoPosition(1);
//                } else {
//                    robotTop.setContainerServoPosition(0.35);
//                }
//                containerRelease = !containerRelease;
//            }
//
//            // robotArm
//            if (gamepad1.x && !previousGamepad1.x) {
//                if (armState == ArmState.IDLE) {
//                    robotTop.setTopServoPosition(0);
//                    armStretchPos = STRETCH_OUT_POSITION;
//                    armState = ArmState.TURNING_OUT;
//                    robotTop.setArmStretchPosition(armStretchPos);
//                } else if (armState == ArmState.TURNED) {
//                    armState = ArmState.TURNING_BACK;
//                    robotTop.setArmLeftSpinPosition(SPIN_Y_DEFAULT_POSITION);
//                }
//            }
//            if (armState == ArmState.WITHDRAWING) {
//                if (armStretchPos <= STRETCH_BACK_POSITION) {
//                    armState = ArmState.IDLE;
//                } else {
//                    armStretchPos -= 0.03;
//                    robotTop.setArmStretchPosition(armStretchPos);
//                }
//            }
//            if (armState == ArmState.TURNING_OUT) {
//                if (armTurnPos >= TURN_OUT_HOVERING_POSITION - 0.05) {
//                    armTurnPos = TURN_OUT_HOVERING_POSITION;
//                    robotTop.setArmLeftTurnPosition(armTurnPos);
//                    robotTop.setArmRightTurnPosition(SPIN_X_HOVERING_POSITION);
//                    armState = ArmState.TURNED;
//                } else {
//                    armTurnPos += 0.03;
//                    robotTop.setArmLeftTurnPosition(armTurnPos);
//                }
//            }
//            if (armState == ArmState.TURNING_BACK) {
//                if (armTurnPos <= TURN_BACK_POSITION + 0.05) {
//                    armTurnPos = TURN_BACK_POSITION;
//                    robotTop.setArmLeftTurnPosition(armTurnPos);
//                    robotTop.setArmRightTurnPosition(SPIN_X_DEFAULT_POSITION);
//                    armState = ArmState.WITHDRAWING;
//                } else {
//                    armTurnPos -= 0.03;
//                    robotTop.setArmLeftTurnPosition(armTurnPos);
//                }
//            }
//            if (armState == ArmState.STRETCHED) {
//                if (gamepad1.x && !previousGamepad1.x) {
//                    robotTop.setTopServoPosition(0);
//                    armState = ArmState.WITHDRAWING;
//                }
//            }
//            if (gamepad1.b && !previousGamepad1.b) {
//                robotTop.setArmGrabPosition(GRAB_OPEN_POSITION);
//                sleep(500);
//                if (!armGrabbing && armState == ArmState.TURNED) {
//                    robotTop.setArmLeftTurnPosition(TURN_OUT_DOWN_POSITION);
//                    robotTop.setArmRightTurnPosition(SPIN_X_DOWN_POSITION);
//                    grabbingFlag = true;
//                } else {
//                    robotTop.setArmGrabPosition(GRAB_CLOSE_POSITION);
//                    armGrabbing = false;
//                }
//            }
//            if (grabbingFlag && !gamepad1.b) {
//                robotTop.setArmGrabPosition(GRAB_CLOSE_POSITION);
//                sleep(500);
//                robotTop.setArmLeftTurnPosition(TURN_OUT_HOVERING_POSITION);
//                robotTop.setArmRightTurnPosition(SPIN_X_HOVERING_POSITION);
//                armGrabbing = true;
//                grabbingFlag = false;
//            }
//            robotTop.setArmStretchPosition(armStretchPos);
//            if (gamepad1.dpad_up) {
//                armSpinXPos = Math.min(1, armSpinXPos + 0.02);
//                robotTop.setArmRightTurnPosition(armSpinXPos);
//            } else if (gamepad1.dpad_down) {
//                armSpinXPos = Math.max(0, armSpinXPos - 0.02);
//                robotTop.setArmRightTurnPosition(armSpinXPos);
//            }
//            if (gamepad1.dpad_right) {
//                armSpinYPos = Math.min(1, armSpinYPos + 0.05);
//                robotTop.setArmLeftSpinPosition(armSpinYPos);
//            } else if (gamepad1.dpad_left) {
//                armSpinYPos = Math.max(0, armSpinYPos - 0.05);
//                robotTop.setArmLeftSpinPosition(armSpinYPos);
//            }
//            if (gamepad1.a && !previousGamepad1.a) {
//                if (backGrabOpen) {
//                    robotTop.setLiftServoPosition(0.1);
//                } else {
//                    robotTop.setLiftServoPosition(0.6);
//                }
//                backGrabOpen = !backGrabOpen;
//            }
//
//            //vision
//            if ((gamepad2.left_trigger != 0 && previousGamepad2.left_trigger == 0
//            ) || (gamepad2.right_trigger != 0 && previousGamepad2.right_trigger == 0)) {
//                recognitionAngle = robotVisionAngle.getDetectedAngle();
//                robotTop.setArmLeftSpinPosition(calculateSpinY(recognitionAngle));
//            }
//
//            // telemetry
//            telemetry.addData("liftPos", liftPosition);
//            telemetry.addData("state", liftState);
//            telemetry.addData("ArmState", armState);
//            telemetry.addData("XPos", armSpinXPos);
//            telemetry.addData("YPos", armSpinYPos);
//            telemetry.addData("angle", recognitionAngle);
//            telemetry.update();
            previousGamepad1.copy(gamepad1);
//            previousGamepad2.copy(gamepad2);
            sleep(10);
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
}