package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.hardware.RobotChassis;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;

@Autonomous
public class AutoTest extends LinearOpMode {
    RobotAuto robotAuto;
    RobotChassis robotChassis;
    RobotTop robotTop;

    @Override
    public void runOpMode() {
        robotChassis = new RobotChassis(this);
        robotAuto = new RobotAuto(this);
        waitForStart();
        resetRuntime();
        robotAuto.turnToHeading(0.7, 45);
        robotChassis.driveRobot(0.6,0,0);
        sleep((int)(24 * 1.414/0.6));
        robotChassis.stopMotor();
        robotAuto.turnToHeading(0.7,180);
        robotTop.setLeftPower(0.6);
        while (robotTop.getLiftPosition() <= 1000){}
        robotTop.setLeftPower(0);
        robotAuto.fastBackward(10);
        robotTop.setLeftPower(-0.3);
        while (robotTop.getLiftPosition() >= 925){}
        robotTop.setLiftServoPosition(0.2);
    }
}